#include "swerve_odom_publisher/swerve_odom_publisher.h"

std::string node_name = "swerve_odom_publisher";

Swerve_Odom_Publisher::Swerve_Odom_Publisher(ros::NodeHandle &nh, const int &loop_rate, const float &body_width, const std::string &base_frame_id)
    : nh_(nh), loop_rate_(loop_rate), BODY_WIDTH(body_width), base_frame_id_(base_frame_id)
{ //constructer, define pubsub
    ROS_INFO("Creating swerve_odom_publisher");
    ROS_INFO_STREAM("loop_rate [Hz]: " << loop_rate_);
    ROS_INFO_STREAM("body_width [m]: " << BODY_WIDTH);
    ROS_INFO_STREAM("base_frame_id: " << base_frame_id_);

    odom_pub = nh_.advertise<nav_msgs::Odometry>("odom", 1);
    sub_RF = nh_.subscribe("data_RF", 1, &Swerve_Odom_Publisher::RF_Callback, this);
    sub_LF = nh_.subscribe("data_LF", 1, &Swerve_Odom_Publisher::LF_Callback, this);
    sub_LB = nh_.subscribe("data_LB", 1, &Swerve_Odom_Publisher::LB_Callback, this);
    sub_RB = nh_.subscribe("data_RB", 1, &Swerve_Odom_Publisher::RB_Callback, this);
    // Float32MultiArray data[2]; data[0]=v, data[1]=theta

    for (int i = 0; i < 4; i++)
    {
        wheelpos[i][0] = BODY_WIDTH / sqrt(2) * cos(M_PI * (2 * i + 1) / 4);
        wheelpos[i][1] = BODY_WIDTH / sqrt(2) * sin(M_PI * (2 * i + 1) / 4);
    }

    update();
}

void Swerve_Odom_Publisher::RF_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    float delta = msg->data[0] * (current_time - last_time).toSec();
    wheelpos[0][0] -= delta * sin(msg->data[1]);
    wheelpos[0][1] += delta * cos(msg->data[1]);
    last_time = current_time;
}

void Swerve_Odom_Publisher::LF_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    float delta = msg->data[0] * (current_time - last_time).toSec();
    wheelpos[1][0] -= delta * sin(msg->data[1]);
    wheelpos[1][1] += delta * cos(msg->data[1]);
    last_time = current_time;
}

void Swerve_Odom_Publisher::LB_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    float delta = msg->data[0] * (current_time - last_time).toSec();
    wheelpos[2][0] -= delta * sin(msg->data[1]);
    wheelpos[2][1] += delta * cos(msg->data[1]);
    last_time = current_time;
}

void Swerve_Odom_Publisher::RB_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    float delta = msg->data[0] * (current_time - last_time).toSec();
    wheelpos[3][0] -= delta * sin(msg->data[1]);
    wheelpos[3][1] += delta * cos(msg->data[1]);
    last_time = current_time;
}

void Swerve_Odom_Publisher::CalcRobotCenter()
{
    center_xy[0] = 0;
    center_xy[1] = 0;
    for (int i = 0; i < 4; i++)
    {
        center_xy[0] += wheelpos[i][0] / 4.0f;
        center_xy[1] += wheelpos[i][1] / 4.0f;
    }
}

void Swerve_Odom_Publisher::CalcRobotAngle()
{
    // 原点に平行移動
    float square_pos[4][2];
    for (int i = 0; i < 4;i++){
        square_pos[i][0] = wheelpos[i][0] - center_xy[0];
        square_pos[i][1] = wheelpos[i][1] - center_xy[1];
    }

    int index;

    // 第一象限に存在する点がただ一つ存在する
    for (int i = 0; i < 4;i++){
        if (square_pos[i][0] > 0 && square_pos[i][1] >= 0){
            index = i;
            break;
        }
    }

    int count = 0;
    theta = 0;
    while(count<4){
        float x = square_pos[index][0];
        float y = square_pos[index][1];

        if(count == 0) // 第1象限
        {
            theta += atan2(y, x) - 45.0f * M_PI / 180.0f;
        }
        else if (count == 1) // 第2象限
        {
            theta += atan2(y, x) - 135.0f * M_PI / 180.0f;
        }
        else if (count == 2) // 第3象限
        {
            theta += atan2(y, x) + 135.0f * M_PI / 180.0f;
        }
        else if (count == 3) // 第4象限
        {
            theta += atan2(y, x) + 45.0f * M_PI / 180.0f;
        }

        index++;
        if(index == 4){
            index = 0;
        }
        count++;
    }
    theta /= 4.0f;

    // odom角に変換
    // 回転
    for (int i = 0; i < 4;i++){
        square_pos[i][0] = square_pos[i][0] / sqrt(2) + square_pos[i][1] / sqrt(2);
        square_pos[i][0] = -1 * square_pos[i][0] / sqrt(2) + square_pos[i][1] / sqrt(2);
    }

    if (square_pos[0][0] > 0 && square_pos[0][1] >= 0); // 第1象限
    else if (square_pos[0][0] <= 0 && square_pos[0][1] > 0) // 第2象限
    {
        theta += 90.0f * M_PI / 180.0f;
    }
    else if (square_pos[0][0] < 0 && square_pos[0][1] <= 0) // 第3象限
    {
        theta -= 180.0f * M_PI / 180.0f;
    }
    else if (square_pos[0][0] >= 0 && square_pos[0][1] < 0) // 第4象限
    {
        theta -= 90.0f * M_PI / 180.0f;
    }
}

void Swerve_Odom_Publisher::update()
{ // if use topic communication
    ros::Rate r(loop_rate_);

    while (ros::ok())
    {
        CalcRobotCenter();
        CalcRobotAngle();
        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        ros::Time current_time = ros::Time::now();
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = base_frame_id_;

        odom_trans.transform.translation.x = center_xy[0];
        odom_trans.transform.translation.y = center_xy[1];
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = center_xy[0];
        odom.pose.pose.position.y = center_xy[1];
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        static ros::Time last_time = ros::Time::now();
        current_time = ros::Time::now();
        float delta_t = (current_time - last_time).toSec();
        odom.child_frame_id = base_frame_id_;
        odom.twist.twist.linear.x = (center_xy[0] - old_center_xy[0]) / delta_t;
        odom.twist.twist.linear.y = (center_xy[1] - old_center_xy[1]) / delta_t;
        odom.twist.twist.angular.z = (theta - old_theta) / delta_t;
        last_time = current_time;

        //publish the message
        odom_pub.publish(odom);

        old_center_xy[0] = center_xy[0];
        old_center_xy[1] = center_xy[1];
        old_theta = theta;

        ros::spinOnce();
        r.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);

    ros::NodeHandle nh;
    ros::NodeHandle arg_n("~");

    int looprate = 30; // Hz
    float body_width = 0.440;
    std::string base_frame_id = "base_link";

    arg_n.getParam("control_frequency", looprate);
    arg_n.getParam("body_width", body_width);
    arg_n.getParam("base_frame_id", base_frame_id);

    Swerve_Odom_Publisher publisher(nh, looprate, body_width, base_frame_id);
    return 0;
}
