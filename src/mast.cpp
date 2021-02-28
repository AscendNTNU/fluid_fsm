/**
 * @file mast.cpp
 */
#include "mast.h"
#include "util.h"
#include "fluid.h"

Mast::Mast(float yaw){
    m_fixed_yaw = yaw;
    m_period = 10;
    m_SHOW_PRINTS = Fluid::getInstance().configuration.interaction_show_prints;
    m_current_extremum = 0;
}

void Mast::update(geometry_msgs::Quaternion orientation){
    const geometry_msgs::Vector3 mast_euler_angle = Util::quaternion_to_euler_angle(orientation);
    m_angle.x =  mast_euler_angle.y;
    m_angle.y = -mast_euler_angle.z;
    m_angle.z =  180.0/M_PI * atan2(mast_euler_angle.y,-mast_euler_angle.z);

    if(m_lookForMin)
    {
        if(m_angle.x < m_current_extremum){
            m_current_extremum = m_angle.x;
            m_time_last_min_pitch = ros::Time::now();
        }
        else if ( (ros::Time::now() - m_time_last_max_pitch) >= ros::Duration(m_period/3.0) ){ //todo: may cause some trouble at the init
            if ((ros::Time::now() - m_time_last_min_pitch) >= ros::Duration(0.5)){ //todo: the last extremum, may be the wrong one, this test is not sufficient
                //We have not found a new minimum for 0.5sec. The last one found it the correct one.
                m_last_min_pitch = m_current_extremum;
                m_lookForMin = false;
                if(!m_time_last_max_pitch.isZero()){ //We have already found a min before
                    m_period = 2* (m_time_last_min_pitch - m_time_last_max_pitch).toSec();
                    if(m_SHOW_PRINTS)
                        ROS_INFO_STREAM("period from min = " << m_period);
                }
            }
        }
    }
    else{ //If we do no look for a min, we look for a max
        if (m_angle.x > m_current_extremum){
            m_current_extremum = m_angle.x;
            m_time_last_max_pitch = ros::Time::now();
        }
        else if ( (ros::Time::now() - m_time_last_min_pitch) >= ros::Duration(m_period/3.0) ){ //todo: may cause some trouble at the init
            if ( (ros::Time::now() - m_time_last_max_pitch) >= ros::Duration(0.5)){ //todo: the last extremum, may be the wrong one, this test is not sufficient
                //We have not found a new maximum for 0.5sec. The last one found it the correct one.
                m_last_max_pitch = m_current_extremum;
                m_lookForMin = true;
                if(!m_time_last_min_pitch.isZero()){ //We have already found a min before
                    m_period = 2 * (m_time_last_max_pitch - m_time_last_min_pitch).toSec();
                    if(m_SHOW_PRINTS)
                        ROS_INFO_STREAM("period from max = " << m_period);
                }
            }
        }
    }
}

float Mast::time_to_max_pitch(){
    // This function assume a triangulare movement of the pitch instead of sinuzoïdal.
    // It works well enough in our case.
    //Todo: could try using arcsin(m_angle.x/m_last_min_pitch), but it creates nan numbers and exceptions.
    // probably not worth the complexity.       
    
    //check that we got both a min and a max
    if(!m_time_last_max_pitch.isZero() && !m_time_last_min_pitch.isZero())
    {
        if(m_lookForMin){
            //the mast is pitching backward:
            return m_period/2.0 *(2.0 - (m_last_max_pitch-m_angle.x)/(m_last_max_pitch-m_last_min_pitch));
        }
        else{
            //the mast is pitching frontward:
            return m_period/2.0 *( 1.0 - (m_angle.x- m_last_min_pitch)/(m_last_max_pitch-m_last_min_pitch));
        }
    }
    return -1;
}


float Mast::get_yaw(){
    return m_fixed_yaw;
}

float Mast::get_period(){
    return m_period;
}