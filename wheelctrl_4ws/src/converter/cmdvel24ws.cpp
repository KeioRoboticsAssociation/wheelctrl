#include "cmdvel24ws.h"
#include "processings.h"

VelConverter::VelConverter(ros::NodeHandle &nh, const float &body_width, const int &lost_time_threshold, const int &loop_rate) 
: nh_(nh), BODY_WIDTH(body_width), lost_time_threshold_(lost_time_threshold), loop_rate_(loop_rate)
{ //constructer, define pubsub
    ROS_INFO("Creating wheelctrl_4ws");
    ROS_INFO_STREAM("body_width [m]: " << BODY_WIDTH);
    ROS_INFO_STREAM("lost_time_threshold [ms]: " << lost_time_threshold_);
    ROS_INFO_STREAM("loop_rate [Hz]: " << loop_rate_);

    init_variables();

    pub_RF = nh_.advertise<std_msgs::Float32MultiArray>(
            "control_RF", 1);
    pub_LF = nh_.advertise<std_msgs::Float32MultiArray>(
        "control_LF", 1);
    pub_LB = nh_.advertise<std_msgs::Float32MultiArray>(
        "control_LB", 1);
    pub_RB = nh_.advertise<std_msgs::Float32MultiArray>(
        "control_RB", 1);
    cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1,
                                 &VelConverter::cmdvelCallback, this);
    bno_sub_ = nh_.subscribe("/pose", 1,
                             &VelConverter::bnoCallback, this);

    last_sub_vel_time_ = std::chrono::system_clock::now();
    last_sub_theta_time_ = std::chrono::system_clock::now();

    update();
}

void VelConverter::init_variables(){
    theta = 0;
    vx = 0;
    vy = 0;
    omega = 0;
    for (int i = 0; i < 4; i++)
    {
        target_speed[i] = 0;
        target_theta[i] = 0;
    }
}

void VelConverter::cmdvel24ws_per_step(const float &theta_body, const float &vx, const float &vy, const float &omega)
{
    static float wheelpos_now[4][2] = {}; //[RF, LF, LB, RB][x, y]
    // present wheel positions
    wheelpos_now[0][0] = BODY_WIDTH / sqrt(2.0f) * cos(theta_body + PI / 4.0f);
    wheelpos_now[0][1] = BODY_WIDTH / sqrt(2.0f) * sin(theta_body + PI / 4.0f);
    wheelpos_now[1][0] = -wheelpos_now[0][1];
    wheelpos_now[1][1] = wheelpos_now[0][0];
    wheelpos_now[2][0] = -wheelpos_now[0][0];
    wheelpos_now[2][1] = -wheelpos_now[0][1];
    wheelpos_now[3][0] = -wheelpos_now[1][0];
    wheelpos_now[3][1] = -wheelpos_now[1][1];

    // next wheel positions
    static float wheelpos_next[4][2] = {};
    wheelpos_next[0][0] = BODY_WIDTH / sqrt(2.0f) * cos(theta_body + omega + PI / 4.0f);
    wheelpos_next[0][1] = BODY_WIDTH / sqrt(2.0f) * sin(theta_body + omega + PI / 4.0f);
    wheelpos_next[1][0] = -wheelpos_next[0][1];
    wheelpos_next[1][1] = wheelpos_next[0][0];
    wheelpos_next[2][0] = -wheelpos_next[0][0];
    wheelpos_next[2][1] = -wheelpos_next[0][1];
    wheelpos_next[3][0] = -wheelpos_next[1][0];
    wheelpos_next[3][1] = -wheelpos_next[1][1];

    for (int i=0;i<4;i++){
        wheelpos_next[i][0] += vx;
        wheelpos_next[i][1] += vy;
    }

    // delta
    static float deltapos[4][2] = {};
    for (int i=0;i<4;i++){// next - now
        deltapos[i][0] = wheelpos_next[i][0] - wheelpos_now[i][0];
        deltapos[i][1] = wheelpos_next[i][1] - wheelpos_now[i][1];
    }

    // calc_angle
    static float wheel_angle[4] = {};
    static float former_wheel_angle[4] = {};
    static int speed_flag[4] = {1, 1, 1, 1};

    for (int i = 0; i < 4; i++)
    { //pay attention
        //atan2
        wheel_angle[0] = atan2(-1*deltapos[i][0], deltapos[i][1]);
        //-pi2pi to -inf2inf
        processings::PI2INF(wheel_angle[i], former_wheel_angle[i]);
        //adjust direction
        processings::AdjustDirection(wheel_angle[i], former_wheel_angle[i], speed_flag[i]);
        former_wheel_angle[i] = wheel_angle[i];
    }

    for (int i = 0; i < 4; i++) {
        // calc target speed (per step) and theta
        target_speed[i] = (float)speed_flag[i] * sqrt(deltapos[i][0] * deltapos[i][0] + deltapos[i][1] * deltapos[i][1]);
        target_theta[i] = wheel_angle[i] - theta_body;
    }
}

void VelConverter::cmdvel24ws(){
    float vx_per_step = vx / (float)loop_rate_;
    float vy_per_step = vy / (float)loop_rate_;
    float omega_per_step = omega / (float)loop_rate_;
    //cmdvel24ws_per_step(theta, vx_per_step, vy_per_step, omega_per_step);
    cmdvel24ws_per_step(0.0, vx_per_step, vy_per_step, omega_per_step);
    // calc target speed (per second)
    for (int i = 0; i < 4; i++) {
        target_speed[i] *= (float)loop_rate_;
    }
}

void VelConverter::reset(){
    ROS_ERROR("wheelctrl_4ws: unable to subscribe topics. Reset velocity...");
    for (int i = 0; i < 4; i++) {
        target_speed[i] = 0;
        // reset velovity, sustain theta
    }
}

void VelConverter::cmdvelCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
    ROS_DEBUG("Received cmd_vel");
    vx = cmd_vel->linear.x;
    vy = cmd_vel->linear.y;
    omega = cmd_vel->angular.z;

    last_sub_vel_time_ = std::chrono::system_clock::now();
}

void VelConverter::bnoCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ROS_DEBUG("Received pose");
    double roll, pitch, yaw;
    processings::geometry_quat_to_rpy(roll, pitch, yaw, msg->pose.orientation);
    theta = yaw;

    last_sub_theta_time_ = std::chrono::system_clock::now();
}

bool VelConverter::isSubscribed() {
    auto current_time = std::chrono::system_clock::now();
    const auto vel_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                             current_time - last_sub_vel_time_).count();
    const auto theta_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                             current_time - last_sub_theta_time_).count();

    if (vel_elapsed < lost_time_threshold_ && theta_elapsed < lost_time_threshold_) {
        return true;
    } else {
        return false;
    }
}

void VelConverter::publishMsg()
{
    std_msgs::Float32MultiArray floatarray;
    floatarray.data.resize(2);
    floatarray.data[0] = target_speed[0];
    floatarray.data[1] = target_theta[0];
    pub_RF.publish(floatarray);

    floatarray.data[0] = target_speed[1];
    floatarray.data[1] = target_theta[1];
    pub_LF.publish(floatarray);

    floatarray.data[0] = target_speed[2];
    floatarray.data[1] = target_theta[2];
    pub_LB.publish(floatarray);

    floatarray.data[0] = target_speed[3];
    floatarray.data[1] = target_theta[3];
    pub_RB.publish(floatarray);
}

void VelConverter::update()
{
    ros::Rate r(loop_rate_);

    while (ros::ok())
    {
        if(isSubscribed()){
            cmdvel24ws();
        }
        else{
            reset();
        }
        publishMsg();
        ros::spinOnce();
        r.sleep();
    }
}
