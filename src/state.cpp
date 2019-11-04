//
// Created by simengangstad on 26.10.18.
//

#include "state.h"

#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ascend_msgs/SplineService.h>

#include <tf/tf.h>

#include <utility>

#include "util.h"
#include "core.h"
#include "PID.h"

fluid::State::State(std::string identifier,
                    std::string px4_mode,
                    bool steady, 
                    bool should_check_obstalce_avoidance_completion) : 

					identifier(identifier),
                    px4_mode(px4_mode),
                    steady(steady), 
                    should_check_obstacle_avoidance_completion(should_check_obstacle_avoidance_completion) {

    pose_subscriber = node_handle.subscribe("mavros/local_position/pose", 1, &State::poseCallback, this);
    twist_subscriber = node_handle.subscribe("mavros/local_position/velocity_local", 1, &State::twistCallback, this);
    imu_subscriber = node_handle.subscribe("mavros/imu/data", 1, &State::imuCallback, this);

    setpoint_publisher = node_handle.advertise<mavros_msgs::PositionTarget>("fluid/setpoint", Core::message_queue_size);

    path_optimizer_client = node_handle.serviceClient<ascend_msgs::PathOptimizerService>("path_optimizer");
}

geometry_msgs::PoseStamped fluid::State::getCurrentPose() const {
	return current_pose;
}

void fluid::State::poseCallback(const geometry_msgs::PoseStampedConstPtr pose) {
    current_pose.pose = pose->pose;
    current_pose.header = pose->header;
}

geometry_msgs::TwistStamped fluid::State::getCurrentTwist() const {
    return current_twist;
}

void fluid::State::twistCallback(const geometry_msgs::TwistStampedConstPtr twist) {
    current_twist.twist = twist->twist;
    current_twist.header = twist->header;
}


void fluid::State::imuCallback(const sensor_msgs::ImuConstPtr imu) {
    current_imu = *imu;
}

void fluid::State::publishSetpoint() {
    setpoint_publisher.publish(setpoint);
}

fluid::ControllerType fluid::State::getPreferredController() const {
    return ControllerType::Passthrough;
}

void fluid::State::perform(std::function<bool(void)> tick, bool should_halt_if_steady) {

    ros::Rate rate(Core::refresh_rate);

    initialize();

    ros::Time startTime = ros::Time::now();
    ros::Time last_frame_time = ros::Time::now();

    fluid::PID yaw_regulator(1.0, 0.0, 0.0);
    const double target_speed = 20.0, curvature_gain = 10.0;

    fluid::Trajectory trajectory(path, getCurrentPose(), getCurrentTwist(), current_imu, target_speed, curvature_gain);


    while (ros::ok() && ((should_halt_if_steady && steady) || !hasFinishedExecution()) && tick()) {

        if (is_relative) {
               
        }
        else {

        }

        yaw_regulator.kp = fluid::Core::yaw_kp;
        yaw_regulator.ki = fluid::Core::yaw_ki;
        yaw_regulator.kd = fluid::Core::yaw_kd;

        ros::Time current_time = ros::Time::now();
        double delta_time = (current_time - last_frame_time).toSec();
        last_frame_time = current_time;

        if (getPreferredController() != ControllerType::Racing) {

            double dx = getCurrentTwist().twist.linear.x;
            double dy = getCurrentTwist().twist.linear.y;

            // Find the current position closest to the path, apply the velocity vector in that direction from the
            // path's yaw and set that as the following point
            double velocity = sqrt(dx*dx + dy*dy);
            ROS_INFO_STREAM(velocity);
            PathPoint current_path_point, following_path_point;
            current_path_point = following_path_point = path.calculateNearestPathPoint(getCurrentPose().pose.position).path_point;
            following_path_point.x += velocity * cos(current_path_point.yaw);
            following_path_point.y += velocity * sin(current_path_point.yaw);

            geometry_msgs::Point point;
            point.x = following_path_point.x;
            point.y = following_path_point.y;
            following_path_point = path.calculateNearestPathPoint(point).path_point;

            tf2::Quaternion quat(getCurrentPose().pose.orientation.x, 
                                 getCurrentPose().pose.orientation.y, 
                                 getCurrentPose().pose.orientation.z, 
                                 getCurrentPose().pose.orientation.w);

            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            // If the quaternion is invalid, e.g. (0, 0, 0, 0), getRPY will return nan, so in that case we just set 
            // it to zero. 
            yaw = std::isnan(yaw) ? 0.0 : yaw;

            // double error = -std::atan(future_path_point.error);
            double error = atan2(following_path_point.y - getCurrentPose().pose.position.y, 
                                 following_path_point.x - getCurrentPose().pose.position.x);

            double steering_yaw = yaw_regulator.getActuation(error, delta_time);
            setpoint.type_mask = TypeMask::Velocity;
            setpoint.yaw = steering_yaw; 
            setpoint.velocity.x = following_path_point.speed * std::cos(setpoint.yaw); 
            setpoint.velocity.y = following_path_point.speed * std::sin(setpoint.yaw);
            setpoint.velocity.z = 0.0;
            setpoint.coordinate_frame = 1; 

            mavros_msgs::PositionTarget yaw_target;
            yaw_target.yaw = error;

            visualizer.publish(getCurrentPose().pose, getCurrentTwist().twist, path, following_path_point, setpoint);
        }
        else {
            setpoint = Core::getControllerPtr()->getSetpoint(getPreferredController(), current_time.toSec(), current_spline);
        }

        publishSetpoint();

        /*

        geometry_msgs::Point spline_position;
        
        spline_position.x = Util::evaluatePolynomial(current_time.toSec(), current_spline.x);
        spline_position.y = Util::evaluatePolynomial(current_time.toSec(), current_spline.y);
        spline_position.z = Util::evaluatePolynomial(current_time.toSec(), current_spline.z);
*/
/*
        if (Util::distanceBetween(spline_position, getCurrentPose().pose.position) > 3.0) {
            // Replan
            startTime = ros::Time::now();
            splines = getSplinesForPath(path);
            current_spline = splines[0];
        }
*/
        /*if (should_check_obstacle_avoidance_completion_ && obstacle_avoidance_completed_) {
            // Obstacle avoidance reported that we've come as far as we can in this state 
            ROS_INFO_STREAM(identifier << ": " << "OA completed");
            break;
        } */        

        fluid::Core::getStatusPublisherPtr()->status.path = this->path;
        fluid::Core::getStatusPublisherPtr()->publish();
        ros::spinOnce();
        rate.sleep();
   }
}