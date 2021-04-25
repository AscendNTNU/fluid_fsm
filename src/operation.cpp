/**
 * @file operation.cpp
 */

#include "operation.h"

#include <mavros_msgs/PositionTarget.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "fluid.h"
#include "util.h"

Operation::Operation(const OperationIdentifier& identifier, const bool& steady, const bool& autoPublish, const bool& isGlobal )
                                        : identifier(identifier), steady(steady), autoPublish(autoPublish), isGlobal(isGlobal){
    
    

    global_pose_subscriber = node_handle.subscribe("mavros/global_position/global", 1, &Operation::globalPoseCallback, this);
    pose_subscriber = node_handle.subscribe("mavros/global_position/local", 1, &Operation::poseCallback, this);
    setpoint_publisher = node_handle.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    global_setpoint_publisher = node_handle.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_raw/global", 10);
    setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    rate_int = (int) Fluid::getInstance().configuration.refresh_rate;
    twist_subscriber = node_handle.subscribe("mavros/local_position/velocity_local", 1, &Operation::twistCallback, this);



}


geometry_msgs::PoseStamped Operation::getCurrentPose() const { return current_pose; }

sensor_msgs::NavSatFix Operation::getGlobalPose() const { return current_global_pose; }

void Operation::poseCallback(const geometry_msgs::PoseStampedConstPtr pose) {
    current_pose.pose = pose->pose;
    current_pose.header = pose->header;
    current_accel = orientation_to_acceleration(pose->pose.orientation);
}

void Operation::globalPoseCallback(const sensor_msgs::NavSatFix pose) {
    current_global_pose.header = pose.header;
    current_global_pose.latitude = pose.latitude;
    current_global_pose.longitude = pose.longitude;
    current_global_pose.altitude = pose.altitude;
    current_global_pose.status = pose.status;
    current_global_pose.position_covariance = pose.position_covariance;
}


geometry_msgs::TwistStamped Operation::getCurrentTwist() const { return current_twist; }


void Operation::twistCallback(const geometry_msgs::TwistStampedConstPtr twist) {
    current_twist.twist = twist->twist;
    current_twist.header = twist->header;
}


geometry_msgs::Vector3 Operation::getCurrentAccel() const { return current_accel; }

geometry_msgs::Vector3 Operation::orientation_to_acceleration(geometry_msgs::Quaternion orientation)
{
    geometry_msgs::Vector3 accel;
    geometry_msgs::Vector3 angle = Util::quaternion_to_euler_angle(orientation);
    accel.x = tan(angle.x) *9.81;
    accel.y = tan(angle.y) *9.81;
    accel.z = 0.0; //we actually don't know ...
    return accel;
}

float Operation::getCurrentYaw() const {
    geometry_msgs::Quaternion quaternion = current_pose.pose.orientation;
    tf2::Quaternion quat(quaternion.x, quaternion.y, quaternion.z, quaternion.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // If the quaternion is invalid, e.g. (0, 0, 0, 0), getRPY will return nan, so in that case we just set
    // it to zero.
    yaw = std::isnan(yaw) ? 0.0 : yaw;

    return yaw;
}

void Operation::publishSetpoint() { 
    if(isGlobal){
        global_setpoint_publisher.publish(global_setpoint);
    }
    else{
        setpoint_publisher.publish(setpoint); 
    }
         
}

void Operation::perform(std::function<bool(void)> should_tick, bool should_halt_if_steady) {

    ros::Rate rate(rate_int);
    initialize();

    do {
        tick();
        if (autoPublish)
            publishSetpoint();
            Fluid::getInstance().getStatusPublisherPtr()->status.setpoint.x = setpoint.position.x;
            Fluid::getInstance().getStatusPublisherPtr()->status.setpoint.y = setpoint.position.y;
            Fluid::getInstance().getStatusPublisherPtr()->status.setpoint.z = setpoint.position.z;
            Fluid::getInstance().getStatusPublisherPtr()->publish();
            ros::spinOnce();
            rate.sleep();
    } 
    while (ros::ok() && ((should_halt_if_steady && steady) || !hasFinishedExecution()) && should_tick());
}