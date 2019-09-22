#ifndef SENSOR_CHECK
#define SENSOR_CHECK

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>


template <typename T, typename Parameter>
class NamedType
{
public:
    explicit NamedType(T const& value) : value_(value) {}
    explicit NamedType(T&& value) : value_(std::move(value)) {}
    T& get() { return value_; }
    T const& get() const {return value_; }
private:
    T value_;
};


using Second = NamedType<float, struct SecondParameter>;
using Timeout = NamedType<Second, struct TimeoutParameter>;

Second operator"" _seconds(long double secs)
{
    return Second(secs);
}

class SensorCheck {
    public:
      SensorCheck(ros::NodeHandle *nh, Timeout timeout);
      bool perform_checks();

    private:
        bool imu_msg_recieved = false;
        bool pressure_msg_recieved = false;

        ros::NodeHandle nh_;
        ros::Duration timeout_duration_;

        ros::Subscriber imu_sub_;
        ros::Subscriber pressure_sub_;

        void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
        void pressure_callback(const sensor_msgs::FluidPressure::ConstPtr& msg);
};
#endif // SENSOR_CHECK
