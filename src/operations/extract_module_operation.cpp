/**
 * @file extract_module_operation.cpp
 */
#include "extract_module_operation.h"

#include "mavros_interface.h"
#include "util.h"

#include <std_srvs/SetBool.h>

//includes to write in a file
#include <iostream>
#include <fstream>
std::ofstream log_drone_position_f; 
const char logFileName[] = "/home/theo/catkin_ws/src/control_pipeline/fluid/log_drone_pos_and_velocity.txt";

void ExtractModuleOperation::initLog()
{ //create a header for the logfile.
    log_drone_position_f.open (logFileName);
    if(log_drone_position_f.is_open())
    {
        log_drone_position_f << "Time\tPos.x\tPos.y\tPos.z\tVel.x\tVel.y\tVel.z\n";
        log_drone_position_f.close();
    }
    else
    {
        ROS_INFO_STREAM(ros::this_node::getName().c_str() << "could not open " << logFileName);
    }
    getCurrentPose();
}

void ExtractModuleOperation::saveLog()
{
    getCurrentPose();
    log_drone_position_f.open (logFileName, std::ios::app); //stored in fluid directory
    if(log_drone_position_f.is_open())
    {
        log_drone_position_f << std::fixed << std::setprecision(3) //only 3 decimals
                           << ros::Time::now() << "\t"
                           << getCurrentPose().pose.position.x << "\t"
                           << getCurrentPose().pose.position.y << "\t"
                           << getCurrentPose().pose.position.z << "\t"
                           << getCurrentTwist().twist.linear.x << "\t"
                           << getCurrentTwist().twist.linear.y << "\t"
                           << getCurrentTwist().twist.linear.z 
                           << "\n";
        log_drone_position_f.close();
    }
}

ExtractModuleOperation::ExtractModuleOperation() : Operation(OperationIdentifier::EXTRACT_MODULE, false) {
    module_pose_subscriber =
        node_handle.subscribe("/sim/module_position", 10, &ExtractModuleOperation::modulePoseCallback, this);
    backpropeller_client = node_handle.serviceClient<std_srvs::SetBool>("/airsim/backpropeller");
}

void ExtractModuleOperation::initialize() {
    MavrosInterface mavros_interface;
    mavros_interface.setParam("MPC_XY_VEL_MAX", speed);
    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Sat speed to: " << speed);

    mavros_interface.setParam("MPC_TILTMAX_AIR", 20);
    mavros_interface.setParam("MPC_Z_VEL_MAX_DN", 0.5);

    // Use the current position as setpoint until we get a message with the module position
    setpoint.position = getCurrentPose().pose.position;
    
    initLog(); //create a header for the logfile.
    previous_time = ros::Time::now();
}

bool ExtractModuleOperation::hasFinishedExecution() const { return module_state == ModuleState::EXTRACTED; }

void ExtractModuleOperation::modulePoseCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr module_pose_ptr) {
    previous_module_pose = module_pose;
    module_pose = *module_pose_ptr;
    ros::Time new_time = ros::Time::now();
    double dt = (new_time - previous_time).nsec;
    module_calculated_velocity.x = (previous_module_pose.pose.pose.position.x - module_pose.pose.pose.position.x)/dt;
    module_calculated_velocity.y = (previous_module_pose.pose.pose.position.y - module_pose.pose.pose.position.y)/dt;
    module_calculated_velocity.z = (previous_module_pose.pose.pose.position.z - module_pose.pose.pose.position.z)/dt;
    previous_time = new_time;
}
/*
void ExtractModuleOperation::calculateModuleVelocity() {
    _velocity_type v = module_pose.pose.pose.position - previous_module_pose.pose.pose.position/
    
}*/


void ExtractModuleOperation::tick() {
    setpoint.type_mask = TypeMask::POSITION_AND_VELOCITY;

    // Wait until we get the first module position readings before we do anything else.
    if (module_pose.header.seq == 0) {
        return;
    }
    
    const double dx = module_pose.pose.pose.position.y - getCurrentPose().pose.position.x;
    const double dy = module_pose.pose.pose.position.x - getCurrentPose().pose.position.y;

    setpoint.yaw = std::atan2(dy, dx) - M_PI / 18.0;

    const double distance_to_module = sqrt(dx * dx + dy * dy);

    const double dvx = getCurrentTwist().twist.linear.x;
    const double dvy = getCurrentTwist().twist.linear.y;
    const double dvz = getCurrentTwist().twist.linear.z;

    const double speed = sqrt(dvx * dvx + dvy * dvy + dvz * dvz);

    switch (module_state) {
        case ModuleState::APPROACHING: {
            setpoint.position.x = module_pose.pose.pose.position.x;
            // TODO: This has to be fixed, should be facing towards the module from any given position,
            // not just from the x direction
            setpoint.position.y = module_pose.pose.pose.position.y; //+ 1.5; //+1.5 removed for testing purposes
            setpoint.position.z = module_pose.pose.pose.position.z;
            setpoint.velocity.x = module_calculated_velocity.x*2;
            setpoint.velocity.y = module_calculated_velocity.y*2;
            setpoint.velocity.z = module_calculated_velocity.z*2;
            ROS_INFO_STREAM(ros::this_node::getName().c_str()
                            << ": "
                            << "Approaching, "
                            << "Curent pose : "
                            << std::fixed << std::setprecision(3) //only 3 decimals
                            << getCurrentPose().pose.position.x
                            << " ; "
                            << getCurrentPose().pose.position.y);
            saveLog();
            //for testing purposes, I remove the possibility to go to the next step
            /*
            if (distance_to_module < 1.8) {
                module_state = ModuleState::OVER;
                ROS_INFO_STREAM(ros::this_node::getName().c_str()
                                << ": "
                                << "Approaching -> Over");
            }
            */
            break;
        }
        case ModuleState::OVER: {
            setpoint.position.x = module_pose.pose.pose.position.y;
            setpoint.position.y = module_pose.pose.pose.position.x + 0.78;
            setpoint.position.z = module_pose.pose.pose.position.z + 0.3;

            const double distance_to_setpoint =
                Util::distanceBetween(setpoint.position, getCurrentPose().pose.position);

            if (distance_to_setpoint < 0.1 && std::abs(getCurrentYaw() - setpoint.yaw) < M_PI / 50.0) {
                module_state = ModuleState::BEHIND_WITH_HOOKS;
            }
            
            break;
        }
        case ModuleState::BEHIND_WITH_HOOKS: {
            setpoint.position.x = module_pose.pose.pose.position.y;
            setpoint.position.y = module_pose.pose.pose.position.x + 0.78;
            setpoint.position.z = module_pose.pose.pose.position.z - 0.1;

            const double distance_to_setpoint =
                Util::distanceBetween(setpoint.position, getCurrentPose().pose.position);

            if (distance_to_setpoint < 0.05 && getCurrentTwist().twist.linear.z < 0.03 && std::abs(getCurrentYaw() - setpoint.yaw) < M_PI / 50.0) {
                module_state = ModuleState::EXTRACTING;
            }

            break;
        }
        case ModuleState::EXTRACTING: {
            setpoint.position.x = module_pose.pose.pose.position.y;
            setpoint.position.y = module_pose.pose.pose.position.x + 2.0;
            setpoint.position.z = module_pose.pose.pose.position.z - 0.1;

            if (!called_backpropeller_service) {
                std_srvs::SetBool request;
                request.request.data = true;
                backpropeller_client.call(request);
                called_backpropeller_service = true;
            }           

            // If the module is on the way down
            // TODO: this should be checked in a better way
            if (module_pose.pose.pose.position.z < 0.5) {
                module_state = ModuleState::EXTRACTED;
                std_srvs::SetBool request;
                request.request.data = false;
                backpropeller_client.call(request);
                called_backpropeller_service = false;
            }

            break;
        }
    }
}
