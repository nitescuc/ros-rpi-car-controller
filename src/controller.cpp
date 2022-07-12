#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Joy.h"
#include "car_actuators/Stop.h"

ros::Publisher steering_publisher, pid_enable_publisher, throttle_publisher;
int steering_axe = 0, throttle_axe = 1, mode_button = 0;
int remote_mode = 0;
double steering = 0.0, throttle = 0.0;
ros::ServiceClient stop_client;

void publish_throttle(double throttle);
void publish_steering(double steering);

bool isAutoSteering() { return remote_mode > -1; }
bool isAutoThrottle() { return remote_mode == 1; }

void enable_pid(bool enabled)
{
    std_msgs::Bool msgs;
    msgs.data = enabled;

    pid_enable_publisher.publish(msgs);
}

void setConfiguration(int in_remote_mode)
{
    car_actuators::Stop stop_req;
    bool change_remote_mode = (remote_mode != in_remote_mode);
    remote_mode = in_remote_mode;

    if (change_remote_mode) 
    {
        publish_throttle(0.0);
        enable_pid(remote_mode == -1);
    }
}

// actuators callback
void remote_callback(const sensor_msgs::Joy joy)
{
    setConfiguration(joy.buttons[0]);
    if (! isAutoThrottle())
    {
        throttle = joy.axes[throttle_axe];
        publish_throttle(throttle);
    }
    if (! isAutoSteering()) 
    {
        steering = joy.axes[steering_axe];
        publish_steering(steering);
    } 
}

void pilot_callback(const sensor_msgs::Joy joy)
{
    if (isAutoSteering()) 
    {
        steering = joy.axes[steering_axe];
        publish_steering(steering);
    }
    if (isAutoThrottle())
    {
        throttle = joy.axes[throttle_axe];
        publish_throttle(throttle);
    }
}

void publish_throttle(double throttle)
{
    std_msgs::Float64 drive;
    drive.data = throttle;

    throttle_publisher.publish(drive);
}

void publish_steering(double steering)
{
    std_msgs::Float64 drive;
    drive.data = steering;
    
    steering_publisher.publish(drive);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;
    ros::NodeHandle pH("~");

    pH.param("axes/steering", steering_axe, 0);
    pH.param("axes/throttle", throttle_axe, 1);
    pH.param("buttons/mode", mode_button, 0);

    throttle_publisher = n.advertise<std_msgs::Float64>("/controller/throttle", 1);
    steering_publisher = n.advertise<std_msgs::Float64>("/controller/steering", 1);
    pid_enable_publisher = n.advertise<std_msgs::Bool>("/pid_enable", 1);

    // Subscribe to /actuator/drive
    ros::Subscriber remote = n.subscribe("/remote", 1, remote_callback);
    ros::Subscriber pilot = n.subscribe("/pilot", 1, pilot_callback);

    enable_pid(true);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
