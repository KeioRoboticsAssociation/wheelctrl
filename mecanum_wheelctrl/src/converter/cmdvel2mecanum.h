#ifndef CMDVEL2MECANUM_H_
#define CMDVEL2MECANUM_H_

#include <ros/ros.h>
#include <iostream>
#include <chrono>

#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>

class VelConverter
{
public:
    VelConverter(ros::NodeHandle &nh, const float &body_height, const float &body_width, const int &lost_time_threshold, const int &loop_rate);
    ~VelConverter(){};

private:
    //Handlers
    ros::NodeHandle &nh_;

    ros::Publisher pub_RF;
    ros::Publisher pub_LF;
    ros::Publisher pub_LB;
    ros::Publisher pub_RB;
    ros::Subscriber cmd_vel_sub_;

    //Configurations
    int loop_rate_;
    float BODY_HEIGHT;
    float BODY_WIDTH;
    int lost_time_threshold_;

    //variables
    float vx;
    float vy;
    float omega;
    float target_speed[4];// 0:RF, 1:LF, 2:LB, 3:RB

    // Timers
    std::chrono::system_clock::time_point last_sub_vel_time_;

    //Methods
    void init_variables();
    void cmdvelCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel);
    bool isSubscribed();
    void publishMsg();
    void cmdvel2mecanum();
    void reset();
    void update();
};

#endif
