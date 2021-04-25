
/**
 * @file move_operation_global.cpp
 */

#include <move_operation_global.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/ParamSet.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <fluid.h>
#include <mavros_interface.h>
#include <util.h>
#include <math.h>



MoveOperationGlobal::MoveOperationGlobal(const OperationIdentifier& operation_identifier,
                             const std::vector<mavros_msgs::GlobalPositionTarget>& global_path, const double& speed,
                             const double& position_threshold, const double& velocity_threshold,
                             const double& max_angle = 45)
    : Operation(operation_identifier, false, true, true),
      path(path),
      speed(speed*100), // Ardupilot operates in cm
      position_threshold(position_threshold),
      velocity_threshold(velocity_threshold),
      max_angle(max_angle*100) {} 
    
bool MoveOperationGlobal::initialize(){
    for (auto iterator = global_path.begin(); iterator!=global_path.end(); iterator++){
        if (iterator->altitude <= 0.1) {
            iterator->altitude = Fluid::getInstance().configuration.default_height;
        }    
    }
    global_setpoint.type_mask = 2048; //Ignore yaw typemask
    been_to_all_points = false;
    current_global_setpoint_iterator = global_path.begin();
    global_setpoint = *current_global_setpoint_iterator;
    
    MavrosInterface mavros_interface;
    mavros_interface.setParam("WPNAV_SPEED", speed);
    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Sat speed to: " << speed/100 << " m/s.");

    mavros_interface.setParam("ANGLE_MAX", max_angle);
    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Sat max angle to: " << max_angle/100 << " deg.");

}
double globalDistance(double lat1, double long1, double alt1, double lat2, double long2, double alt2){
    const double R = 6371000; //radius of earth
    const double phi1 = lat1* boost::math::constants::pi<double>()/180; //convert to radians
    const double phi2 =  lat2* boost::math::constants::pi<double>()/180;
    const double dphi = (lat2 - lat1)* boost::math::constants::pi<double>()/180;
    const double dlambda = (long2 - long1)* boost::math::constants::pi<double>()/180;
    double a = std::sin(dphi/2) * std::sin(dphi/2) +
          std::cos(dphi/2) * std::cos(dphi/2) *
          std::sin(dlambda/2) * std::sin(dlambda/2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
    double d = R*c; //In meters 
    return d;
}
void MoveOperationGlobal::tick(){
    bool at_position_target = globalDistance(getGlobalPose().latitude, getGlobalPose().longitude, 
                                *current_global_setpoint_iterator.latitude, *current_global_setpoint_iterator.longitude) < position_threshold;

    bool low_enough_velocity = std::abs(getCurrentTwist().twist.linear.x) < velocity_threshold &&
                               std::abs(getCurrentTwist().twist.linear.y) < velocity_threshold &&
                               std::abs(getCurrentTwist().twist.linear.z) < velocity_threshold;

    if ((at_position_target && low_enough_velocity) || update_setpoint) {
        if (current_global_setpoint_iterator < global_path.end() - 1) {
            current_global_setpoint_iterator++;

            global_setpoint.position = *current_global_setpoint_iterator;
        
        } else {
            been_to_all_points = true;
        }

        update_setpoint = false;
    }
    
}