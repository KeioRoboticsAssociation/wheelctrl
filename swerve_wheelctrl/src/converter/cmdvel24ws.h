#ifndef CMDVEL24WS_H_
#define CMDVEL24WS_H_

#include <ros/ros.h>
#include <iostream>
#include <chrono>

#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

class VelConverter
{
public:
    VelConverter(ros::NodeHandle &nh, const float &body_width, const int &lost_time_threshold, const int &loop_rate);
    ~VelConverter(){};

private:
    //Handlers
    ros::NodeHandle &nh_;

    ros::Publisher pub_RF;
    ros::Publisher pub_LF;
    ros::Publisher pub_LB;
    ros::Publisher pub_RB;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber bno_sub_;

    //Configurations
    int loop_rate_;
    float BODY_WIDTH;
    int lost_time_threshold_;

    //variables
    float theta;
    float vx;
    float vy;
    float omega;
    float target_speed[4];
    float target_theta[4];

    // Timers
    std::chrono::system_clock::time_point last_sub_vel_time_;
    std::chrono::system_clock::time_point last_sub_theta_time_;

    //Methods
    void init_variables();
    void cmdvelCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel);
    void bnoCallback(const geometry_msgs::PoseStamped::ConstPtr &pose);
    bool isSubscribed();
    void publishMsg();
    void cmdvel24ws_per_step(const float &theta_body, const float &vx, const float &vy, const float &omega);
    void cmdvel24ws();
    void reset();
    void update();
};

#endif
