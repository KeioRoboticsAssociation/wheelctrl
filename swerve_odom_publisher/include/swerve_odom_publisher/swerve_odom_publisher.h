#ifndef SWERVE_ODOM_PUBLISHER_H
#define SWERVE_ODOM_PUBLISHER_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <cmath>
#include <string>

class Swerve_Odom_Publisher
{
public:
    Swerve_Odom_Publisher(ros::NodeHandle &nh, const int &loop_rate, const float &body_width, const std::string &base_farme_id);
    ~Swerve_Odom_Publisher(){};

private:
    //Handlers
    ros::NodeHandle &nh_;

    ros::Publisher odom_pub;
    ros::Subscriber sub_RF;
    ros::Subscriber sub_LF;
    ros::Subscriber sub_LB;
    ros::Subscriber sub_RB;

    tf::TransformBroadcaster odom_broadcaster;

    //Configurations
    int loop_rate_;
    float BODY_WIDTH;
    std::string base_frame_id_;

    //variables
    float wheelpos[4][2]={}; // 0:RF, 1:LF, 2:LB, 3:RB
    float center_xy[2]={};
    float old_center_xy[2]={};
    float theta = 0;
    float old_theta = 0;

    //Methods
    void RF_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void LF_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void RB_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void LB_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void CalcRobotCenter();
    void CalcRobotAngle();
    void update();
};

#endif