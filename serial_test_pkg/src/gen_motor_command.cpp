#include "serial_test_pkg/gen_motor_command.h"

GEN_MOTOR_COMMAND::GEN_MOTOR_COMMAND(ros::NodeHandle &nh, const int &loop_rate, const std::string &mode, const float &max_speed)
    : nh_(nh), loop_rate_(loop_rate), mode_(mode), max_speed_(max_speed)
{
    ROS_INFO("Creating gen_motor_command");

    ROS_INFO_STREAM("loop_rate [Hz]: " << loop_rate_);
    ROS_INFO_STREAM("mode: " << mode_);
    ROS_INFO_STREAM("max_speed: " << max_speed_);

    if(mode_ != "swerve" && mode_ != "mecanum"){
        ROS_ERROR("invalid mode input");
    }

    command_v = 0;
    command_theta = 0;

    control_pub = nh_.advertise<std_msgs::Float32MultiArray>("/control", 1);
    joy_sub = nh_.subscribe("/joy", 1,
                            &GEN_MOTOR_COMMAND::joy_callback, this);
    status_sub = nh_.subscribe("/status", 1,
                               &GEN_MOTOR_COMMAND::status_callback, this);

    update();
}

void GEN_MOTOR_COMMAND::joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg)
{
    if(mode_ == "swerve")
    {
        command_v = joy_msg->axes[JOY_V] * max_speed_;
        command_theta = joy_msg->axes[JOY_THETA] * PI / 2.0;
    }
    else if(mode_ == "mecanum")
    {
        command_v = joy_msg->axes[JOY_THETA] * max_speed_;
    }
}

void GEN_MOTOR_COMMAND::status_callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if(mode_ == "swerve"){
        float v = msg->data[0];
        float theta = msg->data[1];
        ROS_INFO_STREAM("subscribed motor status; v: " << v << " theta: " << theta);
    }
    else if(mode_ == "mecanum"){
        float v = msg->data[0];
        ROS_INFO_STREAM("subscribed motor status; v: " << v);
    }
}

void GEN_MOTOR_COMMAND::publish(void){
    std_msgs::Float32MultiArray floatarray;
    if (mode_ == "swerve")
    {
        floatarray.data.resize(2);
        floatarray.data[0] = command_v;
        floatarray.data[1] = command_theta;
    }
    else if (mode_ == "mecanum")
    {
        floatarray.data.resize(1);
        floatarray.data[0] = command_v;
    }

    control_pub.publish(floatarray);
}

void GEN_MOTOR_COMMAND::update()
{
    ros::Rate r(loop_rate_);

    while (ros::ok())
    {
        publish();
        ros::spinOnce();
        r.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gen_motor_comand");

    ros::NodeHandle nh;
    ros::NodeHandle arg_n("~");

    int looprate = 30; // Hz
    std::string mode = "swerve";
    float max_speed = 1.0;

    arg_n.getParam("controol_frequency", looprate);
    arg_n.getParam("mode", mode);
    arg_n.getParam("max_speed", max_speed);

    GEN_MOTOR_COMMAND gmc(nh, looprate, mode, max_speed);
    return 0;
}
