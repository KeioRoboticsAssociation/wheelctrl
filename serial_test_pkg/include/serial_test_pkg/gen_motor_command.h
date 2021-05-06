#ifndef GEN_MOTOR_COMMAND_H_
#define GEN_MOTOR_COMMAND_H_

#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>

/***************** joystick number ********************/
#define JOY_V 1
#define JOY_THETA 3
/******************************************************/

#define PI 3.141592f

class GEN_MOTOR_COMMAND
{
public:
    GEN_MOTOR_COMMAND(ros::NodeHandle &nh, const int &loop_rate, const std::string &mode, const float &max_speed);
    ~GEN_MOTOR_COMMAND(){};

private:
    //Handlers
    ros::NodeHandle &nh_;

    ros::Publisher control_pub;
    ros::Subscriber joy_sub;
    ros::Subscriber status_sub;

    //Configurations
    int loop_rate_;
    std::string mode_;
    float max_speed_;

    //variables
    float command_v;
    float command_theta;

    //Methods
    void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg);
    void status_callback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void publish(void);
    void update();
};

#endif
