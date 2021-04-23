/**
 * @file fluid.cpp
 */

#include "fluid.h"

#include <fluid/OperationCompletion.h>

#include "explore_operation.h"
#include "interact_operation.h"
#include "fluid.h"
#include "hold_operation.h"
#include "land_operation.h"
#include "mavros_interface.h"
#include "take_off_operation.h"
#include "travel_operation.h"
#include "util.h"

/******************************************************************************************************
 *                                          Singleton                                                 *
 ******************************************************************************************************/

std::shared_ptr<Fluid> Fluid::instance_ptr;

void Fluid::initialize(const FluidConfiguration configuration) {
    if (!instance_ptr) {
        // Can't use std::make_shared here as the constructor is private.
        instance_ptr = std::shared_ptr<Fluid>(new Fluid(configuration));
    }
}

Fluid& Fluid::getInstance() { return *instance_ptr; }

std::shared_ptr<StatusPublisher> Fluid::getStatusPublisherPtr() { return status_publisher_ptr; }

/******************************************************************************************************
 *                                          Operations                                                *
 ******************************************************************************************************/

bool Fluid::take_off(fluid::TakeOff::Request& request, fluid::TakeOff::Response& response) {
    Response attempt_response = attemptToCreateOperation(
        OperationIdentifier::TAKE_OFF,
        {std::make_shared<TakeOffOperation>(request.height), std::make_shared<HoldOperation>()});

    response.message = attempt_response.message;
    response.success = attempt_response.success;
    return true;
}

bool Fluid::travel(fluid::Travel::Request& request, fluid::Travel::Response& response) {
    Response attempt_response =
        attemptToCreateOperation(OperationIdentifier::TRAVEL,
                                 {std::make_shared<TravelOperation>(request.path), std::make_shared<HoldOperation>()});
    
    response.message = attempt_response.message;
    response.success = attempt_response.success;
    return true;
}

bool Fluid::explore(fluid::Explore::Request& request, fluid::Explore::Response& response) {
    Response attempt_response =
        attemptToCreateOperation(OperationIdentifier::EXPLORE,
                                 {std::make_shared<ExploreOperation>(request.path, request.point_of_interest), std::make_shared<HoldOperation>()});

    response.message = attempt_response.message;
    response.success = attempt_response.success;
    return true;
}

bool Fluid::interact(fluid::Interact::Request& request, fluid::Interact::Response& response) {
    Response attempt_response =
        attemptToCreateOperation(OperationIdentifier::INTERACT,
                                 {std::make_shared<InteractOperation>(request.fixed_mast_yaw, request.offset), std::make_shared<HoldOperation>()});
    response.message = attempt_response.message;
    response.success = attempt_response.success;
    return true;
}

bool Fluid::land(fluid::Land::Request& request, fluid::Land::Response& response) {
    Response attempt_response = attemptToCreateOperation(
        OperationIdentifier::LAND, {std::make_shared<LandOperation>(), std::make_shared<LandOperation>()});

    response.message = attempt_response.message;
    response.success = attempt_response.success;
    return true;
}

Fluid::Response Fluid::attemptToCreateOperation(const OperationIdentifier& target_operation_identifier,
                                                const std::list<std::shared_ptr<Operation>>& execution_queue) {
    Response response;
    OperationIdentifier current_operation_identifier = getOperationIdentifierForOperation(current_operation_ptr);

    response.success = isValidOperation(current_operation_identifier, target_operation_identifier);

    if (!response.success) {
        response.message = "Cannot transition to " + getStringFromOperationIdentifier(target_operation_identifier) + 
                " from " + getStringFromOperationIdentifier(current_operation_identifier);
        return response;
    } else {
        ROS_INFO_STREAM(ros::this_node::getName().c_str()
                        << ": "
                        << "Transitioning to "
                        << getStringFromOperationIdentifier(target_operation_identifier).c_str());
    }

    got_new_operation = true;
    operation_execution_queue = execution_queue;
    current_operation = getStringFromOperationIdentifier(operation_execution_queue.front()->identifier);

    return response;
}

std::shared_ptr<Operation> Fluid::performOperationTransition(std::shared_ptr<Operation> current_operation_ptr,
                                                             std::shared_ptr<Operation> target_operation_ptr) {
    ros::Rate rate(Fluid::getInstance().configuration.refresh_rate);
    MavrosInterface mavros_interface;

    // Loop until the Ardupilot mode is set.
    const std::string target_operation_ardupilot_mode = getArdupilotModeForOperationIdentifier(target_operation_ptr->identifier);
    while (ros::ok() && !mavros_interface.attemptToSetMode(target_operation_ardupilot_mode)) {
        ros::spinOnce();
        rate.sleep();
    }

    if (current_operation_ptr) {
        target_operation_ptr->current_pose = current_operation_ptr->getCurrentPose();
        target_operation_ptr->current_twist = current_operation_ptr->getCurrentTwist();
    }

    return target_operation_ptr;
}

/******************************************************************************************************
 *                                          Helpers                                                   *
 ******************************************************************************************************/

OperationIdentifier Fluid::getOperationIdentifierForOperation(std::shared_ptr<Operation> operation_ptr) {
    if (!operation_ptr) {
        return OperationIdentifier::UNDEFINED;
    }

    return operation_ptr->identifier;
}

bool Fluid::isValidOperation(const OperationIdentifier& current_operation_identifier,
                             const OperationIdentifier& target_operation_identifier) const {
    switch (target_operation_identifier) {
        case OperationIdentifier::TAKE_OFF:
            return current_operation_identifier == OperationIdentifier::UNDEFINED ||
                   current_operation_identifier == OperationIdentifier::LAND;

        case OperationIdentifier::EXPLORE:
        case OperationIdentifier::TRAVEL:
            return current_operation_identifier != OperationIdentifier::TAKE_OFF &&
                   current_operation_identifier != OperationIdentifier::LAND &&
                   current_operation_identifier != OperationIdentifier::UNDEFINED;

        case OperationIdentifier::LAND:
            return current_operation_identifier != OperationIdentifier::TAKE_OFF &&
                   current_operation_identifier != OperationIdentifier::UNDEFINED;

        case OperationIdentifier::INTERACT:
            return current_operation_identifier != OperationIdentifier::TAKE_OFF &&
                   current_operation_identifier != OperationIdentifier::LAND &&
                   current_operation_identifier != OperationIdentifier::UNDEFINED;
        default:
            return false;
    }
}

/******************************************************************************************************
 *                                          Main Logic                                                *
 ******************************************************************************************************/

void Fluid::run() {
    ros::Rate rate(configuration.refresh_rate);
    bool has_called_completion = false;

    while (ros::ok()) {
        got_new_operation = false;
        if (!operation_execution_queue.empty()) {
            current_operation_ptr =
                performOperationTransition(current_operation_ptr, operation_execution_queue.front());
            operation_execution_queue.pop_front();
            has_called_completion = false;
        }

        // If we are at the steady operation, we call the completion service
        if (operation_execution_queue.empty() && !has_called_completion) {
            fluid::OperationCompletion operation_completion;
            operation_completion.request.operation = current_operation + " (" + std::to_string(current_operation_ptr->getOperationTag()) + ")";
            operation_completion_client.call(operation_completion);
            has_called_completion = true;
        }

        if (current_operation_ptr) {
            getStatusPublisherPtr()->status.current_operation = current_operation;
            getStatusPublisherPtr()->status.ardupilot_mode =
                getArdupilotModeForOperationIdentifier(current_operation_ptr->identifier);

            current_operation_ptr->perform([&]() -> bool { return !got_new_operation; },
                                           operation_execution_queue.empty());
        }

        ros::spinOnce();
        rate.sleep();
    }
}
