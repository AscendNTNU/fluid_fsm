#include <fluid/Explore.h>
#include <fluid/Interact.h>
#include <fluid/Land.h>
#include <fluid/OperationCompletion.h>
#include <fluid/TakeOff.h>
#include <fluid/Travel.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include <string>
#include <vector>

bool is_executing_operation = false;
std::string finished_operation = "";

/**
 * @brief Grab the finished operation when it completes. This is just a service callback that fluid will
 *        call to. It's not mandatory to implement this service callback, the result of the callback
 *        does not affect the internal state of Fluid, but in that case the client will of course not know when an
 *        operation completes. In other words, just leave this here to have a two-way communication :)
 *
 * @param request Holds the operation completed.
 * @param response Just an empty struct (not used, only required as a parameter for the ROS service).
 *
 * @return true as this service will always succeed, since the Fluid server don't care if this service fails or not
 *              it's just a measure for the two way communication.
 */
bool fluidOperationCompletionCallback(fluid::OperationCompletion::Request& request,
                                      fluid::OperationCompletion::Response& response) {
    finished_operation = request.operation;
    is_executing_operation = false;

    return true;
}

/**
 * @brief Waits for @p timeout until we get connection with the services.
 *
 * @param timeout Time to wait for each service.
 *
 * @return true if we got connetion with all the services.
 */
bool gotConnectionWithServices(const unsigned int& timeout) {
    if (!ros::service::waitForService("fluid/take_off", timeout)) {
        return false;
    }

    if (!ros::service::waitForService("fluid/explore", timeout)) {
        return false;
    }

    if (!ros::service::waitForService("fluid/travel", timeout)) {
        return false;
    }

    if (!ros::service::waitForService("fluid/land", timeout)) {
        return false;
    }

    if (!ros::service::waitForService("fluid/interact", timeout)) {
        return false;
    }

    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fluid_client");
    ros::NodeHandle node_handle;

    // Declared const as we only need to set up the service and get the calls to the callback, we don't need to do
    // anything with this object.
    const ros::ServiceServer operation_completion_server =
        node_handle.advertiseService("fluid/operation_completion", fluidOperationCompletionCallback);

    ros::ServiceClient take_off = node_handle.serviceClient<fluid::TakeOff>("fluid/take_off");
    ros::ServiceClient explore = node_handle.serviceClient<fluid::Explore>("fluid/explore");
    ros::ServiceClient travel = node_handle.serviceClient<fluid::Travel>("fluid/travel");
    ros::ServiceClient land = node_handle.serviceClient<fluid::Land>("fluid/land");
    ros::ServiceClient Interact = node_handle.serviceClient<fluid::Interact>("fluid/interact");

    if (!gotConnectionWithServices(2)) {
        ROS_FATAL("Did not get connection with Fluid's services, is Fluid running?");
        return 1;
    }
    
    fluid::TakeOff take_off_service_handle;
    take_off_service_handle.request.height = 2.0f;

    // In a real implementation the errors should be dealt with in a better manner:
    // 1. Check that the operation can be executed from the current state, in other words check the message from
    //    the response
    // 2. Call the service again after some time interval for example
    if (take_off.call(take_off_service_handle)) {
        if (!take_off_service_handle.response.success) {
            ROS_FATAL_STREAM(take_off_service_handle.response.message);
            return 1;
        }
    } else {
        ROS_FATAL("Failed to call take off service.");
        return 1;
    }

    is_executing_operation = true;

    ros::Rate rate(5);

    while (ros::ok()) {
        if (!is_executing_operation) {
            // Do some more operations when take off has finished
            // This structure is just an example, the logic for when an operation
            // is executing should be implemented in a more secure and bulletproof way
            if (finished_operation == "TAKE_OFF") {
                ROS_INFO_STREAM("[example_client]: Take_off finished, go exploring");
                // Perform an explore with a (list of) point(s)
                geometry_msgs::Point point, POI;
                point.x = 5;
                POI.y = 2;
                fluid::Explore explore_service_handle;
                explore_service_handle.request.path = {point};
                explore_service_handle.request.point_of_interest = POI;
                if (explore.call(explore_service_handle)) {
                    if (!explore_service_handle.response.success) {
                        ROS_FATAL_STREAM(explore_service_handle.response.message);
                        return 1;
                    } else {
                        is_executing_operation = true;
                    }
                } else {
                    ROS_FATAL("Failed to call explore service.");
                    return 1;
                }

            } else if (finished_operation == "EXPLORE") {
                ROS_INFO_STREAM("[example_client]: Exploring finished, go interact with the mast");
                float mast_yaw = M_PI/20.0;
                printf("send yaw mast of %f\n",mast_yaw);
                fluid::Interact interact_service_handle;
                interact_service_handle.request.fixed_mast_yaw = mast_yaw;
                interact_service_handle.request.offset = 1.5;
                
                if (Interact.call(interact_service_handle)) {
                    if (!interact_service_handle.response.success) {
                        ROS_FATAL_STREAM(interact_service_handle.response.message);
                        return 1;
                    } else {
                        is_executing_operation = true;
                    }
                } else {
                    ROS_FATAL("Failed to call interact service.");
                    return 1;
                }
            } else if (finished_operation == "INTERACT") {
                ROS_INFO_STREAM("[example_client]: interaction finished, go travel");
                geometry_msgs::Point point;
                point.x = 100;
                fluid::Travel travel_service_handle;
                travel_service_handle.request.path = {point};
                if (travel.call(travel_service_handle)) {
                    if (!travel_service_handle.response.success) {
                        ROS_FATAL_STREAM(travel_service_handle.response.message);
                        return 1;
                    } else {
                        is_executing_operation = true;
                    }
                } else {
                    ROS_FATAL("Failed to call travel service.");
                    return 1;
                }
            } else if (finished_operation == "TRAVEL") {
                ROS_INFO_STREAM("[example_client]: Traveling finished, Let's land");
                fluid::Land land_service_handle;
                if (land.call(land_service_handle)) {
                    if (!land_service_handle.response.success) {
                        ROS_FATAL_STREAM(land_service_handle.response.message);
                        return 1;
                    } else {
                        is_executing_operation = true;
                    }
                } else {
                    ROS_FATAL("Failed to call land service.");
                    return 1;
                }
            } else if (finished_operation == "LAND") {
                ROS_INFO_STREAM("[example_client]: execution finished. ending process...");
                return 1;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
