//
// Created by simengangstad on 27.09.18.
//

#include <ros/ros.h>

#include "server.h"
#include "core.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "fluid_fsm_server");

    ROS_INFO("Starting Fluid FSM.");

	ros::NodeHandle node_handle;


    node_handle.getParam("refresh_rate", fluid::Core::refresh_rate);
    node_handle.getParam("message_queue_size", fluid::Core::message_queue_size);
    node_handle.getParam("auto_arm", fluid::Core::auto_arm);
    node_handle.getParam("auto_offboard", fluid::Core::auto_set_offboard);

    node_handle.getParam("min_x", fluid::Core::min.x);
    node_handle.getParam("min_y", fluid::Core::min.y);
    node_handle.getParam("min_z", fluid::Core::min.z);
    node_handle.getParam("max_x", fluid::Core::max.z);
    node_handle.getParam("max_y", fluid::Core::max.y);
    node_handle.getParam("max_z", fluid::Core::max.z);

    node_handle.getParam("distance_completion_threshold", fluid::Core::distance_completion_threshold);
    node_handle.getParam("velocity_completion_threshold", fluid::Core::velocity_completion_threshold);
    node_handle.getParam("yaw_completion_threshold", fluid::Core::yaw_completion_threshold);
    node_handle.getParam("default_height", fluid::Core::default_height);
    node_handle.getParam("position_follow_height", fluid::Core::positionFollowHeight);

    fluid::Core::getStatusPublisherPtr()->status.min = fluid::Core::min;
    fluid::Core::getStatusPublisherPtr()->status.min = fluid::Core::max;

    fluid::Server server;
    server.start();

    return 0;
}