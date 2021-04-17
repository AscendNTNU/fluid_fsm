/**
 * @file interact_operation.cpp
 */
#include "interact_operation.h"

#include "mavros_interface.h"
#include "util.h"
#include "fluid.h" //to get access to the tick rate
#include "type_mask.h"

#include <std_srvs/SetBool.h>

//includes to write in a file
#include <iostream>
#include <fstream>
#include <unistd.h> //to get the current directory

//A list of parameters for the user

#define MAST_INTERACT false
    //false blocks the FSM and the drone will NOT properly crash into the mast

#define SAVE_DATA   true
#define SAVE_Z      false
#define USE_SQRT    false
#define ATTITUDE_CONTROL 4   //4 = ignore yaw rate   //Attitude control does not work without thrust

#define TIME_TO_COMPLETION 0.5 //time in second during which we want the drone to succeed a state before moving to the other.
#define APPROACH_ACCURACY 0.06 //Accuracy needed by the drone to go to the next state

// Feedforward tuning
#define ACCEL_FEEDFORWARD_X 0.0
#define ACCEL_FEEDFORWARD_Y 0.0

#define MAX_ANGLE   400 // in centi-degrees


#if SAVE_DATA
//files saved in home directory
const std::string reference_state_path = std::string(get_current_dir_name()) + "/../reference_state.txt";
const std::string drone_pose_path      = std::string(get_current_dir_name()) + "/../drone_state.txt";    
const std::string drone_setpoints_path = std::string(get_current_dir_name()) + "/../drone_setpoints.txt";
std::ofstream reference_state_f; 
std::ofstream drone_pose_f; 
std::ofstream drone_setpoints_f; 
#endif

uint16_t time_cout = 0; //used not to do some stuffs at every tick

//function called when creating the operation
InteractOperation::InteractOperation(const float& fixed_mast_yaw, const float& offset) : 
            Operation(OperationIdentifier::INTERACT, false, false) { 
        mast = Mast(fixed_mast_yaw);
        //Choose an initial offset. It is the offset for the approaching state.
        //the offset is set in the frame of the mast:    
        desired_offset.x = offset;     //forward
        desired_offset.y = 0.0;     //left
        desired_offset.z = -0.45;    //up

    }

void InteractOperation::initialize() {
    //get parameters from the launch file.
    const float* temp = Fluid::getInstance().configuration.LQR_gains;
    for (uint8_t i=0 ; i<2 ; i++) { 
        K_LQR_X[i] = temp[2*i];
        K_LQR_Y[i] = temp[2*i+1];
    }
    SHOW_PRINTS = Fluid::getInstance().configuration.interaction_show_prints;
    GROUND_TRUTH = Fluid::getInstance().configuration.interaction_ground_truth;
    MAX_ACCEL = Fluid::getInstance().configuration.interact_max_acc;
    MAX_VEL = Fluid::getInstance().configuration.interact_max_vel;

    if (GROUND_TRUTH){
        module_pose_subscriber = node_handle.subscribe("/simulator/module/ground_truth/pose",
                                     10, &InteractOperation::modulePoseCallback, this);
    }
    else{
        module_pose_subscriber = node_handle.subscribe("/simulator/module/noisy/pose",
                                     10, &InteractOperation::modulePoseCallback, this);
    }
    // backpropeller_client = node_handle.serviceClient<std_srvs::SetBool>("/airsim/backpropeller");

    attitude_pub = node_handle.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",10);
    //creating own publisher to choose exactly when we send messages
    altitude_and_yaw_pub = node_handle.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);
    attitude_setpoint.type_mask = ATTITUDE_CONTROL;   
    
    setpoint.type_mask = TypeMask::POSITION_AND_VELOCITY;

    MavrosInterface mavros_interface;
    mavros_interface.setParam("ANGLE_MAX", MAX_ANGLE);
    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Sat max angle to: " << MAX_ANGLE/100.0 << " deg.");

    // The desired offset and the transition state are mesured in the mast frame
    transition_state.state.position = desired_offset;
    //transition_state.state.position.y = desired_offset.y;
    //transition_state.state.position.z = desired_offset.z;

    //At the beginning we are far from the mast, we can safely so do a fast transion.
    //But transition state is also the offset, so it should be useless.
    transition_state.cte_acc = 3*MAX_ACCEL; 
    transition_state.max_vel = 3*MAX_VEL;
    completion_count =0;
    faceHugger_is_set = false;    

    //estimation of the time it takes to go from approch state to interact state
    float dist_acc_decc = MAX_VEL*MAX_VEL/MAX_ACCEL;
    estimate_time_to_mast = (desired_offset.x - 0.28 - dist_acc_decc) / MAX_VEL //time during max vel
                            + 2 * MAX_VEL / MAX_ACCEL; //time during acceleration and decceleration
    ROS_INFO_STREAM("estimation of time to mast = " << estimate_time_to_mast);

    #if SAVE_DATA
    //create a header for the datafiles.
    initLog(reference_state_path); 
    initLog(drone_pose_path); 
    initSetpointLog(drone_setpoints_path); 
    #endif

    startApproaching = ros::Time::now();
}

bool InteractOperation::hasFinishedExecution() const { return interaction_state == InteractionState::EXTRACTED; }

void InteractOperation::modulePoseCallback(
    const geometry_msgs::PoseStampedConstPtr module_pose_ptr) {
    previous_module_state = module_state;

    module_state.header = module_pose_ptr->header;
    module_state.position = module_pose_ptr->pose.position;
    module_state.velocity = estimateModuleVel();    
    module_state.acceleration_or_force = estimateModuleAccel();
    
    mast.update(module_pose_ptr->pose.orientation);
}

void InteractOperation::FaceHuggerCallback(const bool released){
    if (released){
        ROS_INFO_STREAM(ros::this_node::getName().c_str() << "CONGRATULATION, FaceHugger set on the mast! We can now exit the mast");
        interaction_state =  InteractionState::EXIT;
        faceHugger_is_set = true;
    }
    else 
        faceHugger_is_set = false;
}

#if SAVE_DATA
void InteractOperation::initSetpointLog(const std::string file_name)
{ //create a header for the logfile.
    std::ofstream save_file_f;
     save_file_f.open(file_name);
    if(save_file_f.is_open())
    {
//        ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": " << file_name << " open successfully");
        save_file_f << "Time\tAccel.x\tAccel.y\n";
        save_file_f.close();
    }
    else
    {
        ROS_INFO_STREAM(ros::this_node::getName().c_str() << "could not open " << file_name);
    }
}

void InteractOperation::saveSetpointLog(const std::string file_name, const geometry_msgs::Vector3 accel)
{
    std::ofstream save_file_f;
    save_file_f.open (file_name, std::ios::app);
    if(save_file_f.is_open())
    {
        save_file_f << std::fixed << std::setprecision(3) //only 3 decimals
                        << ros::Time::now() << "\t"
                        << accel.x << "\t"
                        << accel.y 
                        << "\n";
        save_file_f.close();
    }
}


void InteractOperation::initLog(const std::string file_name)
{ //create a header for the logfile.
    std::ofstream save_file_f;
     save_file_f.open(file_name);
    if(save_file_f.is_open())
    {
//        ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": " << file_name << " open successfully");
        save_file_f << "Time\tPos.x\tPos.y"
            #if SAVE_Z        
            << "\tPos.z"
            #endif
            << "\tVel.x\tVel.y"
            #if SAVE_Z        
            << "\tVel.z"
            #endif
            << "\tAccel.x\tAccel.y"
            #if SAVE_Z        
            << "\tAccel.z";
            #endif
            << "\n";
        save_file_f.close();
    }
    else
    {
        ROS_INFO_STREAM(ros::this_node::getName().c_str() << "could not open " << file_name);
    }
}


void InteractOperation::saveLog(const std::string file_name, const geometry_msgs::PoseStamped pose, const geometry_msgs::TwistStamped vel, const geometry_msgs::Vector3 accel)
{
    std::ofstream save_file_f;
    save_file_f.open (file_name, std::ios::app);
    if(save_file_f.is_open())
    {
        save_file_f << std::fixed << std::setprecision(3) //only 3 decimals
                        << ros::Time::now() << "\t"
                        << pose.pose.position.x << "\t"
                        << pose.pose.position.y << "\t"
                        #if SAVE_Z
                        << pose.pose.position.z << "\t"
                        #endif
                        << vel.twist.linear.x << "\t"
                        << vel.twist.linear.y << "\t"
                        #if SAVE_Z
                        << vel.twist.linear.z << "\t"
                        #endif
                        << accel.x << "\t"
                        << accel.y 
                        #if SAVE_Z
                        << "\t" << accel.z
                        #endif
                        << "\n";
        save_file_f.close();
    }
}

void InteractOperation::saveLog(const std::string file_name, const mavros_msgs::PositionTarget data)
{
    std::ofstream save_file_f;
    save_file_f.open (file_name, std::ios::app);
    if(save_file_f.is_open())
    {
        save_file_f << std::fixed << std::setprecision(3) //only 3 decimals
                        << ros::Time::now() << "\t"
                        << data.position.x << "\t"
                        << data.position.y << "\t"
                        #if SAVE_Z
                        << data.position.z << "\t"
                        #endif
                        << data.velocity.x << "\t"
                        << data.velocity.y << "\t"
                        #if SAVE_Z
                        << data.velocity.z << "\t"
                        #endif
                        << data.acceleration_or_force.x << "\t"
                        << data.acceleration_or_force.y 
                        #if SAVE_Z
                        << "\t" << data.acceleration_or_force.z
                        #endif
                        << "\n";
        save_file_f.close();
    }
}
#endif

geometry_msgs::Vector3 InteractOperation::estimateModuleVel(){
    // estimate the velocity of the module by a simple derivation of the position.
    // In the long run, I expect to receive a nicer estimate by perception or to create a KF myself.
    geometry_msgs::Vector3 vel;
    double dt = (module_state.header.stamp - previous_module_state.header.stamp).nsec/1000000000.0;
    vel.x = (module_state.position.x - previous_module_state.position.x)/dt;
    vel.y = (module_state.position.y - previous_module_state.position.y)/dt;
    vel.z = (module_state.position.z - previous_module_state.position.z)/dt;
    return vel;
}

geometry_msgs::Vector3 InteractOperation::estimateModuleAccel(){
    // estimate the acceleration of the module by simply derivating the velocity.
    // In the long run, I expect to receive a nicer estimate by perception or to createa KF myself.
    geometry_msgs::Vector3 Accel;
    double dt = (module_state.header.stamp - previous_module_state.header.stamp).nsec/1000000000.0;
    Accel.x = (module_state.velocity.x - previous_module_state.velocity.x)/dt;
    Accel.y = (module_state.velocity.y - previous_module_state.velocity.y)/dt;
    Accel.z = (module_state.velocity.z - previous_module_state.velocity.z)/dt;
    return Accel;
}

/*template<typename T>  T& rotate2 (T& pt, float yaw) {
    T& rotated_point;
    rotated_point.x = cos(mast.get_yaw()) * pt.x - sin(mast.get_yaw()) * pt.y;
    rotated_point.y = cos(mast.get_yaw()) * pt.y + sin(mast.get_yaw()) * pt.x;
    rotated_point.z = pt.z;

    return rotated_point;
}
*/
mavros_msgs::PositionTarget InteractOperation::rotate(mavros_msgs::PositionTarget setpoint, float yaw){
    mavros_msgs::PositionTarget rotated_setpoint;
    rotated_setpoint.position = rotate(setpoint.position);
    rotated_setpoint.velocity = rotate(setpoint.velocity);
    rotated_setpoint.acceleration_or_force = rotate(setpoint.acceleration_or_force);

    return rotated_setpoint;
}

geometry_msgs::Vector3 InteractOperation::rotate(geometry_msgs::Vector3 pt, float yaw){
    geometry_msgs::Vector3 rotated_point;
    rotated_point.x = cos(mast.get_yaw()) * pt.x - sin(mast.get_yaw()) * pt.y;
    rotated_point.y = cos(mast.get_yaw()) * pt.y + sin(mast.get_yaw()) * pt.x;
    rotated_point.z = pt.z;

    return rotated_point;
}

geometry_msgs::Point InteractOperation::rotate(geometry_msgs::Point pt, float yaw){
    geometry_msgs::Point rotated_point;
    rotated_point.x = cos(mast.get_yaw()) * pt.x - sin(mast.get_yaw()) * pt.y;
    rotated_point.y = cos(mast.get_yaw()) * pt.y + sin(mast.get_yaw()) * pt.x;
    rotated_point.z = pt.z;

    return rotated_point;
}

geometry_msgs::Vector3 InteractOperation::LQR_to_acceleration(mavros_msgs::PositionTarget ref){
    accel_target.z = 0;
    #if USE_SQRT
        accel_target.x = K_LQR_X[0] * Util::signed_sqrt(ref.position.x - getCurrentPose().pose.position.x) 
                     + K_LQR_X[1] * Util::signed_sqrt(ref.velocity.x - getCurrentTwist().twist.linear.x) 
                     + ACCEL_FEEDFORWARD_X * ref.acceleration_or_force.x;
        accel_target.y = K_LQR_Y[0] * Util::signed_sqrt(ref.position.y - getCurrentPose().pose.position.y) 
                     + K_LQR_Y[1] * Util::signed_sqrt(ref.velocity.y - getCurrentTwist().twist.linear.y) 
                     + ACCEL_FEEDFORWARD_X * ref.acceleration_or_force.y;

    #else
        accel_target.x = K_LQR_X[0] * (ref.position.x - getCurrentPose().pose.position.x) 
                     + K_LQR_X[1] * (ref.velocity.x - getCurrentTwist().twist.linear.x) 
                     + ACCEL_FEEDFORWARD_X * ref.acceleration_or_force.x;
        accel_target.y = K_LQR_Y[0] * (ref.position.y - getCurrentPose().pose.position.y) 
                     + K_LQR_Y[1] * (ref.velocity.y - getCurrentTwist().twist.linear.y) 
                     + ACCEL_FEEDFORWARD_X * ref.acceleration_or_force.y;
    #endif
    // the right of the mast is the left of the drone: the drone is facing the mast
    accel_target.x = - accel_target.x;
    return accel_target;
}

geometry_msgs::Quaternion InteractOperation::accel_to_orientation(geometry_msgs::Vector3 accel){
    double yaw = mast.get_yaw() + M_PI; //we want to face the mast
    double roll = atan2(accel.y,9.81);
    double pitch = atan2(accel.x,9.81);
    return Util::euler_to_quaternion(yaw, roll, pitch);
}

void InteractOperation::update_attitude_input(mavros_msgs::PositionTarget offset){
    mavros_msgs::PositionTarget ref = Util::addPositionTarget(module_state, offset);

    attitude_setpoint.header.stamp = ros::Time::now();
    attitude_setpoint.thrust = 0.5; //this is the thrust that allow a constant altitude no matter what

    accel_target = LQR_to_acceleration(ref);
    attitude_setpoint.orientation = accel_to_orientation(accel_target);
    if(SHOW_PRINTS & time_cout%rate_int==0){
        printf("ref pose\tx %f,\ty %f,\tz %f\n",ref.position.x,
                            ref.position.y, ref.position.z);
    }    

}

void InteractOperation::update_transition_state()
{
// try to make a smooth transition when the relative targeted position between the drone
// and the mast is changed

// Analysis on the x axis
    if (abs(desired_offset.x - transition_state.state.position.x) >= 0.001){
    // if we are in a transition state on the x axis
        transition_state.finished_bitmask &= ~0x1;
        if (Util::sq(transition_state.state.velocity.x) / 2.0 / transition_state.cte_acc 
                            >= abs(desired_offset.x - transition_state.state.position.x))
        {
        // if it is time to brake to avoid overshoot
            //set the transition acceleration (or deceleration) to the one that will lead us to the exact point we want
            transition_state.state.acceleration_or_force.x = - Util::sq(transition_state.state.velocity.x) 
                                            /2.0 / (desired_offset.x - transition_state.state.position.x);
        }
        else if (abs(transition_state.state.velocity.x) > transition_state.max_vel)
        {
        // if we have reached max transitionning speed
            //we stop accelerating and maintain speed
            transition_state.state.acceleration_or_force.x = 0.0;
        }
        else{
        //we are in the acceleration phase of the transition){
            if (desired_offset.x - transition_state.state.position.x > 0.0)
                transition_state.state.acceleration_or_force.x = transition_state.cte_acc;
            else
                transition_state.state.acceleration_or_force.x = - transition_state.cte_acc;
        }
        // Whatever the state we are in, update velocity and position of the target
        transition_state.state.velocity.x = transition_state.state.velocity.x + transition_state.state.acceleration_or_force.x / (float)rate_int;
        transition_state.state.position.x = transition_state.state.position.x + transition_state.state.velocity.x / (float)rate_int;
        
    }
    else if (abs(transition_state.state.velocity.x) < 0.1){
        //setpoint reached destination on this axis
        transition_state.state.position.x = desired_offset.x;
        transition_state.state.velocity.x = 0.0;
        transition_state.state.acceleration_or_force.x = 0.0;
        transition_state.finished_bitmask |= 0x1;
    }

// Analysis on the y axis, same as on the x axis
    if (abs(desired_offset.y - transition_state.state.position.y) >= 0.001){
        transition_state.finished_bitmask = ~0x2;
        if (Util::sq(transition_state.state.velocity.y) / 2.0 / transition_state.cte_acc 
                            >= abs(desired_offset.y - transition_state.state.position.y))
            transition_state.state.acceleration_or_force.y = - Util::sq(transition_state.state.velocity.y) 
                                            /2.0 / (desired_offset.y - transition_state.state.position.y);
        else if (abs(transition_state.state.velocity.y) > transition_state.max_vel)
            transition_state.state.acceleration_or_force.y = 0.0;
        else{
            if (desired_offset.y - transition_state.state.position.y > 0.0)
                transition_state.state.acceleration_or_force.y = transition_state.cte_acc;
            else
                transition_state.state.acceleration_or_force.y = - transition_state.cte_acc;
            }
        transition_state.state.velocity.y  =   transition_state.state.velocity.y  + transition_state.state.acceleration_or_force.y / (float)rate_int;
        transition_state.state.position.y =  transition_state.state.position.y  + transition_state.state.velocity.y / (float)rate_int;
    }
    else if (abs(transition_state.state.velocity.y) < 0.1){
        transition_state.state.position.y = desired_offset.y;
        transition_state.state.velocity.y = 0.0;
        transition_state.state.acceleration_or_force.y = 0.0;
        transition_state.finished_bitmask |= 0x2;
    }

// Analysis on the z axis, same as on the x axis
    if (abs(desired_offset.z - transition_state.state.position.z) >= 0.001){
        transition_state.finished_bitmask = ~0x4;
        if (Util::sq(transition_state.state.velocity.z) / 2.0 / transition_state.cte_acc 
                            >= abs(desired_offset.z - transition_state.state.position.z))
            transition_state.state.acceleration_or_force.z = - Util::sq(transition_state.state.velocity.z) 
                                            /2.0 / (desired_offset.z - transition_state.state.position.z);
        else if (abs(transition_state.state.velocity.z) > transition_state.max_vel)
            transition_state.state.acceleration_or_force.z = 0.0;
        else {
            if (desired_offset.z - transition_state.state.position.z > 0.0)
                transition_state.state.acceleration_or_force.z = transition_state.cte_acc;
            else 
                transition_state.state.acceleration_or_force.z = - transition_state.cte_acc;
        }
        transition_state.state.velocity.z =  transition_state.state.velocity.z + transition_state.state.acceleration_or_force.z / (float)rate_int;
        transition_state.state.position.z =  transition_state.state.position.z + transition_state.state.velocity.z / (float)rate_int;
    }
    else if (abs(transition_state.state.velocity.z) < 0.1){
        transition_state.state.position.z = desired_offset.z;
        transition_state.state.velocity.z = 0.0;
        transition_state.state.acceleration_or_force.z = 0.0;
        transition_state.finished_bitmask |= 0x4;
    }
}

void InteractOperation::tick() {
    time_cout++;
    //printf("mast pitch %f, roll %f, angle %f\n", mast_angle.x, mast_angle.y, mast_angle.z);
    // Wait until we get the first module position readings before we do anything else.
    if (module_state.header.seq == 0) {
        if(time_cout%rate_int==0)
            printf("Waiting for callback\n");
        startApproaching = ros::Time::now();
        return;
    }

    if(SHOW_PRINTS)
        if(time_cout%rate_int == 0)
            ROS_INFO_STREAM("max pitch ETA: " << ros::Time::now() + ros::Duration(mast.time_to_max_pitch()));

    update_transition_state();
    geometry_msgs::Point rotated_offset = rotate(desired_offset,mast.get_yaw());

    const double dx = module_state.position.x + rotated_offset.x - getCurrentPose().pose.position.x;
    const double dy = module_state.position.y + rotated_offset.y - getCurrentPose().pose.position.y;
    const double dz = module_state.position.z + rotated_offset.z - getCurrentPose().pose.position.z;
    const double distance_to_offset = sqrt(Util::sq(dx) + Util::sq(dy) + Util::sq(dz));
    

    switch (interaction_state) {
        case InteractionState::APPROACHING: {
            if (SHOW_PRINTS){
                if(time_cout%rate_int==0) {
                    printf("APPROACHING\t");
                    printf("distance to ref %f\n", distance_to_offset);
                }
            }
            float time_out_gain = 1 + (ros::Time::now()-startApproaching).toSec()/30.0;
            if(MAST_INTERACT) {
                if ( distance_to_offset <= APPROACH_ACCURACY *time_out_gain ) { 
                    //Todo, we may want to judge the velocity in stead of having a time to completion
                    if (completion_count < ceil(TIME_TO_COMPLETION * (float)rate_int)-1 )
                        completion_count++;
                    else if(mast.time_to_max_pitch() !=-1){
                        //We consider that if the drone is ready at some point, it will 
                        // remain ready until it is time to try
                        completion_count = 0;
                        float time_to_wait = mast.time_to_max_pitch()-estimate_time_to_mast;
                        if(time_to_wait < TIME_WINDOW_INTERACTION)
                            time_to_wait += mast.get_period();
                        ROS_INFO_STREAM(ros::this_node::getName().c_str() 
                                << ": Ready to set the FaceHugger. Waiting for the best opportunity"
                                << "\nEstimated waiting time before go: "
                                << time_to_wait);
                        interaction_state = InteractionState::READY;                
                    }
                }
                else
                    completion_count = 0;
            }
            break;
        }
        case InteractionState::READY: {
            //The drone is ready, we just have to wait for the best moment to go!
            float time_to_wait = mast.time_to_max_pitch()-estimate_time_to_mast;
            if (SHOW_PRINTS){
                if(time_cout%(rate_int/2)==0) {
                    ROS_INFO_STREAM("READY; "
                            << "Estimated waiting time before go: "
                            << time_to_wait);
                }
            }
            if( abs(time_to_wait+TIME_WINDOW_INTERACTION/2.0) <= TIME_WINDOW_INTERACTION/2.0)
            { //We are in the good window to set the faceHugger
                interaction_state = InteractionState::OVER;
                ROS_INFO_STREAM(ros::this_node::getName().c_str()
                            << ": " << "Approaching -> Over");
                //the offset is set in the frame of the mast:    
                desired_offset.x = 0.28;  //forward
                desired_offset.y = 0.0;   //left
                desired_offset.z = -0.45; //up
                transition_state.cte_acc = MAX_ACCEL;
                transition_state.max_vel = MAX_VEL;

                // Avoid going to the next step before the transition is actuallized
                transition_state.finished_bitmask = 0;
            }

            break;
        }
        case InteractionState::OVER: {
            if (SHOW_PRINTS){
                if(time_cout%(rate_int*2)==0) printf("OVER\n");
            }
            //We assume that the accuracy is fine, we don't want to take the risk to stay too long
            if (transition_state.finished_bitmask & 0x7 == 0x7) {
                interaction_state = InteractionState::INTERACT;
                ROS_INFO_STREAM(ros::this_node::getName().c_str()
                            << ": " << "Over -> Interact");
                desired_offset.x = 0.28;  //forward
                desired_offset.y = 0.0;   //left
                desired_offset.z -= 0.2;  //up

                // Avoid going to the next step before the transition is actuallized
                transition_state.finished_bitmask = 0;
            }
            break;
        }
        case InteractionState::INTERACT: {
            if (SHOW_PRINTS){
                if(time_cout%(rate_int*2)==0) printf("INTERACT\n");
            }

            // we don't want to take the risk to stay too long, 
            // Whether the faceHugger is set or not, we have to exit.
            // NB, when FH is set, an interupt function switches the state to EXIT
            if (transition_state.finished_bitmask & 0x7 == 0x7) {
                interaction_state = InteractionState::EXIT;
                ROS_INFO_STREAM(ros::this_node::getName().c_str() 
                        << "Interact -> Exiting\n"
                        << "Exit for safety reasons, the FaceHugger could not be placed..."); 

                //we move backward to ensure there will be no colision
                // We directly set the transition state as we want to move as fast as possible
                // and we don't mind anymore about the relative position to the mast
                transition_state.state.position.x = 2.70;  //further than the desired offset as a fix to make it faster
                transition_state.state.position.y = 0.0;   
                transition_state.state.position.z = -1.0;  
                desired_offset.x = 1.70;   //forward
                desired_offset.y = 0.0;    //left
                desired_offset.z = -0.8;   //up

                //todo: for some reason, the drone is slow to get there. 
                // It would be nice to get it go back to a stable approach within 5 secs
                // so that we can try to set the FaecHugger as soon as possible!
                // It may be because of the LQR control which is tuned for accuracy and not fast movements.
                // It can be overriden by setting far setpoints, but that's not clean, and may include risks
            }
            break;
        }
        case InteractionState::EXIT: {
            // NB, when FH is set, an interupt function switches the state to EXIT
            #if SHOW_PRINTS
            if(time_cout%(rate_int*2)==0) printf("EXIT\n");
            #endif
            //This is a transition state before going back to approach and try again.
            if ( distance_to_offset < 0.6 ) {
                if (faceHugger_is_set){
                    ROS_INFO_STREAM(ros::this_node::getName().c_str()
                            << ": " << "Exit -> Extracted");
                    interaction_state = InteractionState::EXTRACTED;
                    desired_offset.x = 2;
                    desired_offset.y = 0.0;
                    desired_offset.z = 3;
                }
                else {
                    //we may want to reset startApproaching, but I don't think so
                    ROS_INFO_STREAM(ros::this_node::getName().c_str()
                            << ": " << "Exit -> Approaching");
                    interaction_state = InteractionState::APPROACHING;
                    desired_offset.x = 1.5;
                    desired_offset.y = 0.0;
                    desired_offset.z = -0.45;
                }
            }
            break;
        }
        case InteractionState::EXTRACTED: {
            #if SHOW_PRINTS
            if(time_cout%(rate_int*2)==0) printf("EXIT\n");
            #endif
            // This is also a transition state before AI takes the lead back and travel back to the starting point
            std_srvs::SetBool request;
            request.request.data = false; 
            if ( distance_to_offset < 0.2 ) {
                desired_offset.x = 2;
                desired_offset.y = 0.0;
                desired_offset.z = 3;
            }
        }



    }//end switch state

    if (time_cout % rate_int == 0)
    {
        if (SHOW_PRINTS){
        //    printf("desired offset \t\tx %f, y %f, z %f\n",desired_offset.x,
        //                    desired_offset.y, desired_offset.z);
            printf("transition pose\tx %f,\ty %f,\tz %f\n",transition_state.state.position.x,
                            transition_state.state.position.y, transition_state.state.position.z);
        //    printf("transition vel\t x %f, y %f, z %f\n",transition_state.state.velocity.x,
        //                    transition_state.state.velocity.y, transition_state.state.velocity.z);
        //    printf("transition accel\t x %f, y %f, z %f\n",transition_state.state.acceleration_or_force.x,
        //                    transition_state.state.acceleration_or_force.y, transition_state.state.acceleration_or_force.z);
            geometry_msgs::Point cur_drone_pose = getCurrentPose().pose.position;
            printf("Drone pose\tx %f,\ty %f,\tz %f\tyaw %f\n",cur_drone_pose.x,
                                         cur_drone_pose.y, cur_drone_pose.z,getCurrentYaw());
            printf("Accel target\tx %f,\ty %f,\tz %f\n",accel_target.x,
                                         accel_target.y, accel_target.z);
        }
    }

    mavros_msgs::PositionTarget smooth_rotated_offset = rotate(transition_state.state,mast.get_yaw());
    update_attitude_input(smooth_rotated_offset);

    if (time_cout % 2 == 0)
    {
        // todo: it may be possible to publish more often without any trouble.
        setpoint.header.stamp = ros::Time::now();
        setpoint.yaw = mast.get_yaw()+M_PI;
        setpoint.position.x = module_state.position.x + smooth_rotated_offset.position.x;
        setpoint.position.y = module_state.position.y + smooth_rotated_offset.position.y;
        setpoint.position.z = module_state.position.z + smooth_rotated_offset.position.z;
        setpoint.velocity.x = module_state.velocity.x + smooth_rotated_offset.velocity.x;
        setpoint.velocity.y = module_state.velocity.y + smooth_rotated_offset.velocity.y;
        setpoint.velocity.z = module_state.velocity.z + smooth_rotated_offset.velocity.z;

        altitude_and_yaw_pub.publish(setpoint);

    }

    attitude_pub.publish(attitude_setpoint);

    #if SAVE_DATA
    //save the control data into files
    saveLog(reference_state_path,Util::addPositionTarget(module_state, smooth_rotated_offset));
    saveLog(drone_pose_path, getCurrentPose(),getCurrentTwist(),getCurrentAccel());
    saveSetpointLog(drone_setpoints_path,accel_target);
    #endif

}
