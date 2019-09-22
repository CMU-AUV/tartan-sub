#include "sensor_check.h"
#include <ros/ros.h>
#include <iostream>


int main(int argc, char **argv) {
    ros::init(argc, argv, "SenorChecker");

    std::cout<<"Performing Sensor Check"<<std::endl;
    ros::NodeHandle nh;

    SensorCheck sensor_checker = SensorCheck(&nh, Timeout(10.0_seconds));

    if(sensor_checker.perform_checks()){
        std::cout<<"Sensor Checks Passed"<<std::endl;
    }
    else{
        std::cout<<"Sensor Checks Falied"<<std::endl;
    }
    return 0;
}
