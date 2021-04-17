#include "status_publisher.h"

StatusPublisher::StatusPublisher() {
    status.armed = 0;
    status.linked_with_ardupilot = 0;
    status.ardupilot_mode = "none";
    status.current_operation = "none";

    pose_subscriber = node_handle.subscribe("mavros/local_position/pose", 1, &StatusPublisher::poseCallback, this);
    status_publisher = node_handle.advertise<ascend_msgs::FluidStatus>("fluid/status", 1);
    trace_publisher = node_handle.advertise<nav_msgs::Path>("fluid/trace", 1);
    setpoint_marker_publisher = node_handle.advertise<visualization_msgs::Marker>("fluid/setpoint_setpoint_marker", 10);

    setpoint_marker.header.frame_id = "/map";
    setpoint_marker.header.stamp = ros::Time::now();

    setpoint_marker.id = 9999;
    setpoint_marker.type = visualization_msgs::Marker::SPHERE;
    setpoint_marker.action = visualization_msgs::Marker::ADD;

    setpoint_marker.pose.orientation.w = 1.0;
    setpoint_marker.scale.x = setpoint_marker.scale.y = setpoint_marker.scale.z = 0.2;

    setpoint_marker.color.r = 1.0f;
    setpoint_marker.color.g = 0.0f;
    setpoint_marker.color.b = 1.0f;
    setpoint_marker.color.a = 1.0;

    setpoint_marker.lifetime = ros::Duration();
}

void StatusPublisher::poseCallback(const geometry_msgs::PoseStampedConstPtr pose_ptr) {
    if (trace_path.poses.size() > 300) {
        trace_path.poses.erase(trace_path.poses.begin());
    }

    trace_path.poses.push_back(*pose_ptr);
}

void StatusPublisher::publish() {
    status_publisher.publish(status);
    trace_path.header.stamp = ros::Time::now();
    trace_path.header.frame_id = "map";
    trace_publisher.publish(trace_path);

    setpoint_marker.pose.position.x = status.setpoint.x;
    setpoint_marker.pose.position.y = status.setpoint.y;
    setpoint_marker.pose.position.z = status.setpoint.z;

    setpoint_marker_publisher.publish(setpoint_marker);
}
