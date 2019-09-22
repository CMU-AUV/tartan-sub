#include "sensor_check.h"
#include "ros/time.h"
#include <iostream>

constexpr float rate = 10.0;

// ################################################################################################ //

SensorCheck::SensorCheck(ros::NodeHandle *nh, Timeout timeout)
    :nh_(*nh)
{   auto seconds = timeout.get();
    timeout_duration_ = ros::Duration(seconds.get()/rate, 0.0);
    imu_sub_ = nh_.subscribe("/rexrov/imu", rate, &SensorCheck::imu_callback, this);
    pressure_sub_ = nh_.subscribe("/rexrov/pressure", rate, &SensorCheck::pressure_callback, this);
}

// ################################################################################################ //

void SensorCheck::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_msg_recieved = true;
    return;
}

// ################################################################################################ //

void SensorCheck::pressure_callback(const sensor_msgs::FluidPressure::ConstPtr& msg)
{
    pressure_msg_recieved = true;
    return;
}

// ################################################################################################ //

bool SensorCheck::perform_checks()
{
    for( auto i = 0; i < 10; i++){
        if(pressure_msg_recieved & imu_msg_recieved)
            return true;
        timeout_duration_.sleep();
        ros::spinOnce();
    }
    return false;
}

// ################################################################################################ //
