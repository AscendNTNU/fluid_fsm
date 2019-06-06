//
// Created by simengangstad on 08.11.18.
//

#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/String.h>
#include <fluid/OperationGoal.h>

#include <cmath>
#include <assert.h>
#include <algorithm>

#include "server.h"
#include "core.h"
#include "pose_util.h"

fluid::Server::Server() : actionlib_server_(node_handle_, 
                                            "fluid_fsm_operation",
                                            false) {
    
    actionlib_server_.registerGoalCallback(boost::bind(&Server::goalCallback, this));
    actionlib_server_.registerPreemptCallback(boost::bind(&Server::preemptCallback, this));

    actionlib_server_.start();
}


void fluid::Server::goalCallback() {

    // We accept the new goal and initialize variables for target pose and the type of operation identifier.
    // This is necessary in order to modify some of them before we initiate the different operations further down.
    // E.g. the init operation shouldn't be called with a different pose than (0, 0, 0), so we make sure this is the
    // case.
    
    if (actionlib_server_.isActive()) {
        actionlib_server_.setAborted();
    }

    auto goal = actionlib_server_.acceptNewGoal();
    mavros_msgs::PositionTarget setpoint = goal->setpoint;
    std::string destination_identifier = goal->type.data;
    timeout_ = ros::Duration(goal->timeout.data);

    // Check first if a boundry is defined (!= 0). If there is a boundry the position target is clamped to 
    // min and max.
    if (fluid::Core::min.x != 0 || fluid::Core::max.x != 0) {
        setpoint.position.x = std::max(fluid::Core::min.x, 
                                       std::min(static_cast<float>(setpoint.position.x), fluid::Core::max.x));
    }

    if (fluid::Core::min.y != 0 || fluid::Core::max.y != 0) { 
        setpoint.position.y = std::max(fluid::Core::min.y, 
                                       std::min(static_cast<float>(setpoint.position.y), fluid::Core::max.y));
    }

    if (fluid::Core::min.z != 0 || fluid::Core::max.z != 0) {
        setpoint.position.z = std::max(fluid::Core::min.z, 
                                       std::min(static_cast<float>(setpoint.position.z), fluid::Core::max.z));
    }

    Core::getStatusPublisherPtr()->status.setpoint = setpoint;


    bool shouldIncludeMove = PoseUtil::distanceBetween(fluid::Core::getGraphPtr()->current_state_ptr->getCurrentPose(), setpoint) >= fluid::Core::distance_completion_threshold;

    auto states = fluid::Core::getGraphPtr()->getPathToEndState(fluid::Core::getGraphPtr()->current_state_ptr->identifier, destination_identifier, shouldIncludeMove);
    ROS_INFO_STREAM("New operation requested to transition to state: " << destination_identifier);

    std::stringstream stringstream;

    for (auto state : states) {
        stringstream << state->identifier << " ";
    }

    ROS_INFO_STREAM("Will traverse through: " << stringstream.str());


    next_operation_p_ = std::make_shared<fluid::Operation>(destination_identifier, setpoint);

    new_operation_requested_ = true;
}

void fluid::Server::preemptCallback() {
    actionlib_server_.setPreempted();
}

void fluid::Server::start() {

    ros::Rate rate(fluid::Core::refresh_rate);

    // Main loop of Fluid FSM. This is where all the magic happens. If a new operaiton is requested, the 
    // new operation requested flag is set and we set up the requirements for that operation to run. When it
    // it runs we check every tick if a new operation is requested and abort from the current operation if
    // that is the case. 
    while (ros::ok()) {

        // Setup for the new operation.
        if (new_operation_requested_) {
            current_operation_p_ = next_operation_p_;
            next_operation_p_.reset();
            new_operation_requested_ = false;
        }

        // Execute the operation if there is any
        if (current_operation_p_) {

            ros::Time start_time = ros::Time::now();

            fluid::Core::getStatusPublisherPtr()->status.current_operation = current_operation_p_->getDestinationStateIdentifier();

            current_operation_p_->perform(

                [&]() -> bool {
                    return new_operation_requested_ || (ros::Time::now() - start_time > timeout_); 
                },

                [&](bool completed) {
                    // We completed the operation and want to end at the final state of the operation (e.g. hold)
                    // state for move. One can think of this step as making sure that the state machine is at a
                    // state where it's easy to execute a new operation. If we did not complete the operation has
                    // already transitioned to a steady state and we just set the last state to that state.s
                    if (completed) {
                        last_state_p_ = current_operation_p_->getFinalStatePtr();
                    }
                    else {
                        last_state_p_ = fluid::Core::getGraphPtr()->current_state_ptr;
                    }


                    // Will notify the operation client what the outcome of the operation was. This will end up
                    // calling the callback that the operation client set up for completion.
                    
                    if (!actionlib_server_.isActive()) {
                        return;
                    }

                    if (completed) {
                        ROS_INFO_STREAM("Operation completed.");
                        actionlib_server_.setSucceeded();
                    }
                    else {
                        actionlib_server_.setAborted();
                    }
                });

            current_operation_p_.reset();
        }
        // We don't have a current operation, so we just continue executing the last state.
        else {

            fluid::Core::getStatusPublisherPtr()->status.current_operation = "none";

            if (last_state_p_) {

                last_state_p_->perform([&]() -> bool {
                    // We abort the execution of the current state if there is a new operation.
                    return new_operation_requested_;
                }, true);
            }
        }

        fluid::Core::getStatusPublisherPtr()->publish();

        ros::spinOnce();
        rate.sleep();
    }
}