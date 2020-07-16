#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

ros::Publisher drive_publisher, pilot_publisher, *crt_publisher;
int steering_axe = 0, throttle_axe = 1, mode_button = 0;
int remote_mode = 0, pilot_mode = 0;
double attenuation = 0.8;
double steering = 0.0, throttle = 0.0;

void publish(ros::Publisher *publisher, double steering, double throttle);

bool isAutoSteering() { return pilot_mode == 1; }
bool isAutoThrottle() { return pilot_mode == 1 &&  remote_mode == 1; }
bool isThrottleAttenuated() { return remote_mode != 1; }

void setConfiguration(int in_pilot_mode, int in_remote_mode)
{
    pilot_mode = in_pilot_mode;
    remote_mode = in_remote_mode;
    if (isAutoThrottle())
    {
        crt_publisher = &pilot_publisher;
    } else {
        crt_publisher = &drive_publisher;
    }
}

// actuators callback
void remote_callback(const sensor_msgs::Joy joy)
{
    setConfiguration(pilot_mode, joy.buttons[0]);
    if (! isAutoThrottle())
    {
        throttle = joy.axes[throttle_axe];
        if (isThrottleAttenuated())
        {
            throttle = throttle * attenuation;
        }
    }
    if (! isAutoSteering()) 
    {
        steering = joy.axes[steering_axe];

        publish(crt_publisher, steering, throttle);
    } 
}

void pilot_callback(const sensor_msgs::Joy joy)
{
    setConfiguration(joy.buttons[0], remote_mode);
    if (isAutoSteering()) 
    {
        steering = joy.axes[steering_axe];
        if (isAutoThrottle())
        {
            throttle = joy.axes[throttle_axe];
        }
        publish(crt_publisher, steering, throttle);
    }
}

void publish(ros::Publisher *publisher, double steering, double throttle)
{
    geometry_msgs::Twist drive;
    drive.angular.z = steering;
    drive.linear.x = throttle;
    
    publisher->publish(drive);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;
    ros::NodeHandle pH("~controller");

    pH.param("axes/steering", steering_axe, 0);
    pH.param("axes/throttle", throttle_axe, 1);
    pH.param("buttons/mode", mode_button, 0);
    pH.param("attenuation", attenuation, 0.8);

    drive_publisher = n.advertise<geometry_msgs::Twist>("/actuator/drive", 1);
    pilot_publisher = n.advertise<geometry_msgs::Twist>("/actuator/pilot", 1);

    // Subscribe to /actuator/drive
    ros::Subscriber remote = n.subscribe("/remote", 1, remote_callback);   
    ros::Subscriber pilot = n.subscribe("/pilot", 1, pilot_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
