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
#define SAVE_DATA   true
#define SAVE_Z      false
#define USE_SQRT    false
#define ATTITUDE_CONTROL 4   //4 = ignore yaw rate   //Attitude control does not work without thrust

#define TIME_TO_COMPLETION 0.5 //time in second during which we want the drone to succeed a state before moving to the other.


// Feedforward tuning
#define ACCEL_FEEDFORWARD_X 0.0
#define ACCEL_FEEDFORWARD_Y 0.0

#define MAX_ANGLE   400 // in centi-degrees


//mast movement estimation
#define SAVE_PITCH_TIME 15
#define SAVE_PITCH_FREQ 4 //Todo, may not work with whatever frequency. 6Hz looks weird

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
            Operation(OperationIdentifier::INTERACT, false, false), fixed_mast_yaw(fixed_mast_yaw) 
    { 
        //Choose an initial offset. It is the offset for the approaching state.
        //the offset is set in the frame of the mast:    
        desired_offset.x = offset;     //forward
        desired_offset.y = 0.0;     //left
        desired_offset.z = -0.45;    //up

    }

void InteractOperation::initialize() {
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

    mast_pitches = (float*) calloc(SAVE_PITCH_FREQ*SAVE_PITCH_TIME,sizeof(float));
    mast_pitches_id=0;

//    printf("\n\n save rate = %d\n\n",rate_int/SAVE_PITCH_FREQ);

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
    
    
    #if SAVE_DATA
    //create a header for the datafiles.
    initLog(reference_state_path); 
    initLog(drone_pose_path); 
    initSetpointLog(drone_setpoints_path); 
    #endif
}

bool InteractOperation::hasFinishedExecution() const { return interaction_state == InteractionState::EXTRACTED; }

void InteractOperation::modulePoseCallback(
    const geometry_msgs::PoseStampedConstPtr module_pose_ptr) {
    previous_module_state = module_state;

    module_state.header = module_pose_ptr->header;
    module_state.position = module_pose_ptr->pose.position;
    module_state.velocity = estimateModuleVel();    
    module_state.acceleration_or_force = estimateModuleAccel();
    
    const geometry_msgs::Vector3 mast_euler_angle = Util::quaternion_to_euler_angle(module_pose_ptr->pose.orientation);
    mast_angle.x =  mast_euler_angle.y;
    mast_angle.y = -mast_euler_angle.z;
    mast_angle.z =  180.0/M_PI * atan2(mast_euler_angle.y,-mast_euler_angle.z);
}

void InteractOperation::FaceHuggerCallback(const bool released){
    if (released){
        ROS_INFO_STREAM(ros::this_node::getName().c_str() << "CONGRATULATION, FaceHugger set on the mast! We can now leave the mast");
        interaction_state =  InteractionState::LEAVE;
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

void InteractOperation::save_mast_pitch(int save_rate){
    mast_pitches[mast_pitches_id] = mast_angle.x;
    mast_pitches_id++;
//    printf("mast angle n°%d: %f\n",mast_pitches_id,mast_angle.x);
    if(mast_pitches_id==SAVE_PITCH_FREQ*SAVE_PITCH_TIME){
        mast_pitches_id=0;
        estimate_mast_period(save_rate);
    }
}

int InteractOperation::search_min_id_within(float* array, int begin, int end){
    int min_id = begin;
    for(int i = begin ; i < end ; i++)
    {
        if(array[i] <= array[min_id])
            min_id = i;
    }
    return min_id;
}

int InteractOperation::search_max_id_within(float* array, int begin, int end){
    int max_id = begin;
    for(int i = begin ; i < end ; i++)
    {
        if(array[i] >= array[max_id])
            max_id = i;
    }
    return max_id;
}

void InteractOperation::estimate_mast_period(int save_rate){
    // perception should get it with the kalman filter, but nice to have it ourselves I guess

    // This first version consider that the newest values are the lowest indexes.
    // It also consider that we never get out of bounds

    // We try to find the min and the max looking at successif short intervals
    // By taking 2 seconds interval, we know that we won't have more than a full
    // period, and therefore, we can't have two local extremum.
    // If the indice of the extremum is the beginning indice or the end indice
    // that means that we probably did not find it.
    int min_id=0, max_id=0;
    int interval_end;
    //First, we estimate whether the pitches is increasing or decreasing
    if(mast_pitches[0] > mast_pitches[5]) {
        // pitches in decreasing, we first look for the min
        do {
            interval_end = min_id+3*save_rate;
            min_id = search_min_id_within(mast_pitches,min_id,interval_end);
        } while (min_id >= interval_end-save_rate/2-1); // -1 because the last indice is not included in the previous search
        // let us look for the max now
        max_id = min_id + 3*save_rate; //save some iterations
        do {
            interval_end = max_id+3*save_rate;
            max_id = search_max_id_within(mast_pitches,max_id,interval_end);
        } while (max_id >= interval_end-save_rate/2-1); // -1 because the last indice is not included in the previous search
                                                    // -save_rate/2 to take some margin as the signal may be noisy
        
    }
    else {
        // pitches in decreasing, we first look for the min
        do {
            interval_end = max_id+3*save_rate;
            max_id = search_max_id_within(mast_pitches,max_id,interval_end);
        } while (max_id >= interval_end-save_rate/2-1); // -1 because the last indice is not included in the previous search
        // let us look for the min now
        min_id = max_id + 3*save_rate; //save some iterations
        do {
            interval_end = min_id+3*save_rate;
            min_id = search_min_id_within(mast_pitches,min_id,interval_end);
        } while (min_id >= interval_end-save_rate/2-1); // -1 because the last indice is not included in the previous search
    }
    float half_period = (float)abs(min_id - max_id)/(float)save_rate;
    if (SHOW_PRINTS){
        printf("The mast period is %f\n\n", 2*half_period);
    }
}


/*template<typename T>  T& rotate2 (T& pt, float yaw) {
    T& rotated_point;
    rotated_point.x = cos(fixed_mast_yaw) * pt.x - sin(fixed_mast_yaw) * pt.y;
    rotated_point.y = cos(fixed_mast_yaw) * pt.y + sin(fixed_mast_yaw) * pt.x;
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
    rotated_point.x = cos(fixed_mast_yaw) * pt.x - sin(fixed_mast_yaw) * pt.y;
    rotated_point.y = cos(fixed_mast_yaw) * pt.y + sin(fixed_mast_yaw) * pt.x;
    rotated_point.z = pt.z;

    return rotated_point;
}

geometry_msgs::Point InteractOperation::rotate(geometry_msgs::Point pt, float yaw){
    geometry_msgs::Point rotated_point;
    rotated_point.x = cos(fixed_mast_yaw) * pt.x - sin(fixed_mast_yaw) * pt.y;
    rotated_point.y = cos(fixed_mast_yaw) * pt.y + sin(fixed_mast_yaw) * pt.x;
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
    double yaw = fixed_mast_yaw + M_PI; //we want to face the mast
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
        return;
    }
    //printf("time count %% 5 = %d\n", (int)(time_cout%5));
    if((int)time_cout % (rate_int/SAVE_PITCH_FREQ ) ==0){
        save_mast_pitch(rate_int/SAVE_PITCH_FREQ);
    }
    
    update_transition_state();
    mavros_msgs::PositionTarget smooth_rotated_offset = rotate(transition_state.state,fixed_mast_yaw);

    const double dx = module_state.position.x + smooth_rotated_offset.position.x - getCurrentPose().pose.position.x;
    const double dy = module_state.position.y + smooth_rotated_offset.position.y - getCurrentPose().pose.position.y;
    const double dz = module_state.position.z + smooth_rotated_offset.position.z - getCurrentPose().pose.position.z;
    const double distance_to_reference_with_offset = sqrt(Util::sq(dx) + Util::sq(dy) + Util::sq(dz));
    

    switch (interaction_state) {
        case InteractionState::APPROACHING: {
            if (SHOW_PRINTS){
                if(time_cout%rate_int==0) {
                    printf("APPROACHING\t");
                    printf("distance to ref %f\n", distance_to_reference_with_offset);
                }
            }
            if ((transition_state.finished_bitmask & 0x7 == 0x7) && (distance_to_reference_with_offset < 0.07)) {
                if (completion_count <= ceil(TIME_TO_COMPLETION*(float) rate_int) )
                    completion_count++;
                else{
                    interaction_state = InteractionState::OVER;
                    ROS_INFO_STREAM(ros::this_node::getName().c_str()
                                << ": " << "Approaching -> Over");
                    //the offset is set in the frame of the mast:    
                    desired_offset.x = 0.28;  //forward   //right //the distance from the drone to the FaceHugger
                    desired_offset.y = 0.0;   //left   //front
                    desired_offset.z = -0.45;  //up   //up
                    transition_state.cte_acc = MAX_ACCEL;
                    transition_state.max_vel = MAX_VEL;
                    completion_count= 0;

                    // Avoid going to the next step before the transition is actuallized
                    transition_state.finished_bitmask = 0;

                }
            }
            else
                completion_count = 0;
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
                desired_offset.x = 0.28;  //forward    //the distance from the drone to the FaceHugger
                desired_offset.y = 0.0;   //left      
                desired_offset.z -= 0.2; //up        // going down by 20 cms

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
            // Whether the faceHugger is set or not, we have to leave.
            // NB, when FH is set, an interupt function switches the state to LEAVE
            if (transition_state.finished_bitmask & 0x7 == 0x7) {
                interaction_state = InteractionState::LEAVE;
                ROS_INFO_STREAM(ros::this_node::getName().c_str() << "Leaving the mast for safety reasons, the FaceHugger could not be placed..."); 

                //we move backward to ensure there will be no colision
                // We directly set the transition state as we want to move as fast as possible
                // and we don't mind anymore about the relative position to the mast
                transition_state.state.position.x = 1.70;  //forward   //right //the distance from the drone to the FaceHugger
                transition_state.state.position.y = 0.0;   //left      //front
                transition_state.state.position.z = -0.8;  //up        //up
                desired_offset.x = 1.70;  //forward    //the distance from the drone to the FaceHugger
                desired_offset.y = 0.0;   //left      
                desired_offset.z = -0.8; //up        // going down by 20 cms
            }
            break;
        }
        case InteractionState::LEAVE: {
            // NB, when FH is set, an interupt function switches the state to LEAVE
            #if SHOW_PRINTS
            if(time_cout%(rate_int*2)==0) printf("LEAVE\n");
            #endif
            //This is a transition state before going back to approach and try again.
            if (distance_to_reference_with_offset < 0.2) {
                if (faceHugger_is_set){
                    interaction_state = InteractionState::EXTRACTED;
                    desired_offset.x = 2;
                    desired_offset.y = 0.0;
                    desired_offset.z = 3;
                }
                else {
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
            if(time_cout%(rate_int*2)==0) printf("LEAVE\n");
            #endif
            // This is also a transition state before AI takes the lead back and travel back to the starting point
            std_srvs::SetBool request;
            request.request.data = false; 
            if (distance_to_reference_with_offset < 0.2) {
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

    update_attitude_input(smooth_rotated_offset);

    if (time_cout % 2 == 0)
    {
        // todo: it may be possible to publish more often without any trouble.
        setpoint.header.stamp = ros::Time::now();
        setpoint.yaw = fixed_mast_yaw+M_PI;
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