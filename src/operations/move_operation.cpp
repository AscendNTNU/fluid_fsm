/**
 * @file move_operation.cpp
 */

#include "move_operation.h"

#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/ParamSet.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>

#include "fluid.h"
#include "mavros_interface.h"
#include "util.h"

MoveOperation::MoveOperation(const OperationIdentifier& operation_identifier,
                             const std::vector<geometry_msgs::Point>& path, const double& speed,
                             const double& position_threshold, const double& velocity_threshold,
                             const double& max_angle = 45)
    : Operation(operation_identifier, false, true),
      path(path),
      speed(speed*100),
      position_threshold(position_threshold),
      velocity_threshold(velocity_threshold),
      max_angle(max_angle*100) {}

bool MoveOperation::hasFinishedExecution() const { return been_to_all_points; }

void MoveOperation::initialize() {
    for (auto iterator = path.begin(); iterator != path.end(); iterator++) {
        if (iterator->z <= 0.1) {
            iterator->z = Fluid::getInstance().configuration.default_height;
        }
    }

    setpoint.type_mask = TypeMask::POSITION_AND_VELOCITY;
    been_to_all_points = false;
    current_setpoint_iterator = path.begin();
    setpoint.position = *current_setpoint_iterator;

    double dx = current_setpoint_iterator->x - getCurrentPose().pose.position.x;
    double dy = current_setpoint_iterator->y - getCurrentPose().pose.position.y;
    setpoint.yaw = std::atan2(dy, dx);
    
    
    MavrosInterface mavros_interface;
    mavros_interface.setParam("WPNAV_SPEED", speed);
    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Sat speed to: " << speed/100 << " m/s.");

    mavros_interface.setParam("ANGLE_MAX", max_angle);
    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Sat max angle to: " << max_angle/100 << " deg.");

}

void MoveOperation::tick() {
    bool at_position_target =
        Util::distanceBetween(getCurrentPose().pose.position, *current_setpoint_iterator) < position_threshold;
    bool low_enough_velocity = std::abs(getCurrentTwist().twist.linear.x) < velocity_threshold &&
                               std::abs(getCurrentTwist().twist.linear.y) < velocity_threshold &&
                               std::abs(getCurrentTwist().twist.linear.z) < velocity_threshold;

    if ((at_position_target && low_enough_velocity) || update_setpoint) {
        if (current_setpoint_iterator < path.end() - 1) {
            current_setpoint_iterator++;

            setpoint.position = *current_setpoint_iterator;

            double dx = current_setpoint_iterator->x - getCurrentPose().pose.position.x;
            double dy = current_setpoint_iterator->y - getCurrentPose().pose.position.y;
            setpoint.yaw = std::atan2(dy, dx);
        
        } else {
            been_to_all_points = true;
        }

        update_setpoint = false;
    }
}

