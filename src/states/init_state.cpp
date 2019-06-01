//
//  Created by Simen Gangstad on 15/10/2018.
//


#include "../../include/states/init_state.h"

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ascend_msgs/FluidFsmStatus.h>

#include "../../include/core/core.h"
#include "../../include/mavros/mavros_state_setter.h"
#include "../../include/mavros/mavros_state_subscriber.h"
#include "../../include/mavros/type_mask.h"

bool fluid::InitState::hasFinishedExecution() {
    return initialized;
}

void fluid::InitState::tick() {
    // Not implemented as all logic happens inside perform for the init state has we have to arm and set offboard mode
}

void fluid::InitState::perform(std::function<bool (void)> shouldAbort) {

    ros::Rate rate(Core::refresh_rate);
    ros::NodeHandle node_handle_;

    ascend_msgs::FluidFsmStatus status;

    status.min_x = fluid::Core::minX;
    status.min_y = fluid::Core::minY;
    status.min_z = fluid::Core::minZ;
    status.max_x = fluid::Core::maxX;
    status.max_y = fluid::Core::maxY;
    status.max_z = fluid::Core::maxZ;

    fluid::Core::getStatusPublisherPtr()->status = status;

    // Establishing contact through mavros with Pixhawk.

    fluid::MavrosStateSetter state_setter(Core::message_queue_size, 
    									  1.0/static_cast<double>(Core::refresh_rate), 
    									  "OFFBOARD");

    ROS_INFO("Attempting to establish contact with PX4...");

    // Run until we achieve a connection with mavros
    while (ros::ok() && !state_setter.getCurrentState().connected) {
        fluid::Core::getStatusPublisherPtr()->publish();
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("OK!\n");

    fluid::Core::getStatusPublisherPtr()->status.linked_with_px4 = 1;

    //send a few setpoints before starting. This is because the stream has to be set ut before we
    // change modes within px4
    setpoint.position.x = 0;
    setpoint.position.y = 0;
    setpoint.position.z = 0;
    setpoint.type_mask = fluid::TypeMask::Idle;

    for (int i = Core::refresh_rate*2; ros::ok() && i > 0; --i) {
        setpoint_publisher.publish(setpoint);
        fluid::Core::getStatusPublisherPtr()->publish();
        ros::spinOnce();
        rate.sleep();
    }


    // Arming
    ROS_INFO_STREAM("Attemping to arm...");

    if (!fluid::Core::auto_arm) {
        ROS_INFO("Waiting for arm signal...");
    }


    ros::Time last_request = ros::Time::now();
    ros::ServiceClient arming_client = node_handle_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mavros_msgs::CommandBool arm_command;
    arm_command.request.value = true;
    bool armed = false;
    double arm_request_interval = 0.5;

    while (ros::ok() && !hasFinishedExecution() && !armed) {

        // Send request to arm every interval specified
        if (ros::Time::now() - last_request > ros::Duration(arm_request_interval)) {

            if (!state_setter.getCurrentState().armed) {
                if (fluid::Core::auto_arm) {
                    if(arming_client.call(arm_command) && arm_command.response.success) {
                        fluid::Core::getStatusPublisherPtr()->status.armed = 1;
                        armed = true;
                    }
                }
            }
            else {
                fluid::Core::getStatusPublisherPtr()->status.armed = 1;
                armed = true;
            }

            last_request = ros::Time::now();
        }

        fluid::Core::getStatusPublisherPtr()->publish();
        setpoint_publisher.publish(setpoint);

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("OK!\n");


    // Offboard
    ROS_INFO("Trying to set offboard...");

    if (!Core::auto_set_offboard) {
        ROS_INFO("Waiting for offboard signal...");        
    }


    fluid::MavrosStateSubscriber state_subscriber;

    bool set_offboard = false;

    while(ros::ok() && !hasFinishedExecution() && !set_offboard) {

        set_offboard = state_subscriber.getCurrentState().mode == "OFFBOARD";

        if (Core::auto_set_offboard) { 
            state_setter.attemptToSetState([&](bool completed) {

                set_offboard = completed;

                if (completed) {
                    fluid::Core::getStatusPublisherPtr()->status.px4_mode = "offboard";
                }
            });
        }
        

        fluid::Core::getStatusPublisherPtr()->publish();
        setpoint_publisher.publish(setpoint);

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("OK!\n");

    initialized = true;
}
