//
// Created by simengangstad on 27.09.18.
//

#include <ros/ros.h>

#include "server.h"
#include "core.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "fluid_fsm_server");

    ROS_INFO("Starting Fluid FSM.");

    fluid::Core::refresh_rate = static_cast<unsigned int>(atoi(argv[1]));
    fluid::Core::message_queue_size = static_cast<unsigned int>(atoi(argv[2]));
    fluid::Core::auto_arm = std::string(argv[3]) == "true";
    fluid::Core::auto_set_offboard = std::string(argv[4]) == "true";
    
	ros::NodeHandle node_handle;
    
    node_handle.getParam("minX", fluid::Core::minX);
    node_handle.getParam("minY", fluid::Core::minY);
    node_handle.getParam("minZ", fluid::Core::minZ);
    node_handle.getParam("maxX", fluid::Core::maxX);
    node_handle.getParam("maxY", fluid::Core::maxY);
    node_handle.getParam("maxZ", fluid::Core::maxZ);

    node_handle.getParam("distanceCompletionThreshold", fluid::Core::distance_completion_threshold);
    node_handle.getParam("velocityCompletionThreshold", fluid::Core::velocity_completion_threshold);
    node_handle.getParam("yawCompletionThreshold", fluid::Core::yaw_completion_threshold);
    node_handle.getParam("defaultHeight", fluid::Core::default_height);
    node_handle.getParam("positionFollowHeight", fluid::Core::positionFollowHeight);

    fluid::Server server;
    server.start();

    return 0;
}
