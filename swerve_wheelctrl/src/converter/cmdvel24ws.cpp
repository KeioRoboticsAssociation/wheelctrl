#include "cmdvel24ws.h"
#include "processings.h"

VelConverter::VelConverter(ros::NodeHandle &nh, const double &body_width, const int &lost_time_threshold, const int &loop_rate, const bool &gazebo_mode) 
: nh_(nh), BODY_WIDTH(body_width), lost_time_threshold_(lost_time_threshold), loop_rate_(loop_rate), gazebo_mode_(gazebo_mode)
{ //constructer, define pubsub
    ROS_INFO("Creating swerve_wheelctrl");
    ROS_INFO_STREAM("body_width [m]: " << BODY_WIDTH);
    ROS_INFO_STREAM("lost_time_threshold [ms]: " << lost_time_threshold_);
    ROS_INFO_STREAM("loop_rate [Hz]: " << loop_rate_);
    ROS_INFO_STREAM("gazebo_mode: " << gazebo_mode_);

    init_variables();

    if(gazebo_mode_){
        pub_RF = nh_.advertise<std_msgs::Float64>(
            "control_RF", 1);
        pub_LF = nh_.advertise<std_msgs::Float64>(
            "control_LF", 1);
        pub_LB = nh_.advertise<std_msgs::Float64>(
            "control_LB", 1);
        pub_RB = nh_.advertise<std_msgs::Float64>(
            "control_RB", 1);
        pub_RF_angle = nh_.advertise<std_msgs::Float64>(
            "control_RF_angle", 1);
        pub_LF_angle = nh_.advertise<std_msgs::Float64>(
            "control_LF_angle", 1);
        pub_LB_angle = nh_.advertise<std_msgs::Float64>(
            "control_LB_angle", 1);
        pub_RB_angle = nh_.advertise<std_msgs::Float64>(
            "control_RB_angle", 1);
    }
    else{
        pub_RF = nh_.advertise<std_msgs::Float32MultiArray>(
            "control_RF", 1);
        pub_LF = nh_.advertise<std_msgs::Float32MultiArray>(
            "control_LF", 1);
        pub_LB = nh_.advertise<std_msgs::Float32MultiArray>(
            "control_LB", 1);
        pub_RB = nh_.advertise<std_msgs::Float32MultiArray>(
            "control_RB", 1);
    }

    cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1,
                                 &VelConverter::cmdvelCallback, this);

    last_sub_vel_time_ = std::chrono::system_clock::now();

    update();
}

void VelConverter::init_variables(){
    vx_ = 0;
    vy_ = 0;
    omega_ = 0;
    for (int i = 0; i < 4; i++)
    {
        target_speed[i] = 0;
        target_theta[i] = 0;
    }
}

void VelConverter::cmdvel24ws_per_step(const double &vx, const double &vy, const double &omega)
{
    static double wheelpos_now[4][2] = {}; //[RF, LF, LB, RB][x, y]
    // present wheel positions
    wheelpos_now[0][0] = BODY_WIDTH / 2.0f;
    wheelpos_now[0][1] = BODY_WIDTH / 2.0f;
    wheelpos_now[1][0] = -wheelpos_now[0][1];
    wheelpos_now[1][1] = wheelpos_now[0][0];
    wheelpos_now[2][0] = -wheelpos_now[0][0];
    wheelpos_now[2][1] = -wheelpos_now[0][1];
    wheelpos_now[3][0] = -wheelpos_now[1][0];
    wheelpos_now[3][1] = -wheelpos_now[1][1];

    // next wheel positions
    static double wheelpos_next[4][2] = {};
    wheelpos_next[0][0] = BODY_WIDTH / sqrt(2.0f) * cos(omega + M_PI / 4.0f);
    wheelpos_next[0][1] = BODY_WIDTH / sqrt(2.0f) * sin(omega + M_PI / 4.0f);
    wheelpos_next[1][0] = -wheelpos_next[0][1];
    wheelpos_next[1][1] = wheelpos_next[0][0];
    wheelpos_next[2][0] = -wheelpos_next[0][0];
    wheelpos_next[2][1] = -wheelpos_next[0][1];
    wheelpos_next[3][0] = -wheelpos_next[1][0];
    wheelpos_next[3][1] = -wheelpos_next[1][1];

    for (int i=0;i<4;i++){
        wheelpos_next[i][0] -= vy;
        wheelpos_next[i][1] += vx;
    }

    // delta
    static double deltapos[4][2] = {};
    for (int i=0;i<4;i++){// next - now
        deltapos[i][0] = wheelpos_next[i][0] - wheelpos_now[i][0];
        deltapos[i][1] = wheelpos_next[i][1] - wheelpos_now[i][1];
    }

    // calc_angle
    static double wheel_angle[4] = {};
    static double former_wheel_angle[4] = {};
    static int speed_flag[4] = {1, 1, 1, 1};

    for (int i = 0; i < 4; i++)
    { //pay attention
        //atan2
        wheel_angle[i] = atan2(-1*deltapos[i][0], deltapos[i][1]);
        
        //-pi2pi to -inf2inf
        processings::PI2INF(wheel_angle[i], former_wheel_angle[i]);
        //adjust direction
        processings::AdjustDirection(wheel_angle[i], former_wheel_angle[i], speed_flag[i]);
        former_wheel_angle[i] = wheel_angle[i];
    }

    for (int i = 0; i < 4; i++) {
        // calc target speed (per step) and theta
        target_speed[i] = (double)speed_flag[i] * sqrt(deltapos[i][0] * deltapos[i][0] + deltapos[i][1] * deltapos[i][1]);
        target_theta[i] = wheel_angle[i];
        //ROS_INFO("%i, %f", i, target_theta[i]);
    }
}

void VelConverter::cmdvel24ws(){
    double vx_per_step = vx_ / (double)loop_rate_;
    double vy_per_step = vy_ / (double)loop_rate_;
    double omega_per_step = omega_ / (double)loop_rate_;
    //cmdvel24ws_per_step(vx_per_step, vy_per_step, omega_per_step);
    cmdvel24ws_per_step(vx_per_step, vy_per_step, omega_per_step);
    // calc target speed (per second)
    for (int i = 0; i < 4; i++) {
        target_speed[i] *= (double)loop_rate_;
    }
}

void VelConverter::reset(){
    ROS_ERROR("swerve_wheelctrl: unable to subscribe topics. Reset velocity...");
    for (int i = 0; i < 4; i++) {
        target_speed[i] = 0;
        // reset velovity, sustain theta
    }
}

void VelConverter::cmdvelCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
    ROS_DEBUG("Received cmd_vel");
    vx_ = cmd_vel->linear.x;
    vy_ = cmd_vel->linear.y;
    omega_ = cmd_vel->angular.z;

    last_sub_vel_time_ = std::chrono::system_clock::now();
}

bool VelConverter::isSubscribed() {
    auto current_time = std::chrono::system_clock::now();
    const auto vel_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                             current_time - last_sub_vel_time_).count();

    if (vel_elapsed < lost_time_threshold_) {
        return true;
    } else {
        return false;
    }
}

void VelConverter::publishMsg()
{
    if(gazebo_mode_){
        std_msgs::Float64 command;
        // rad/s
        command.data = target_speed[0] / 0.035; // devide by wheel radius to convert m/s to rad/s
        pub_RF.publish(command);
        command.data = target_theta[0];
        pub_RF_angle.publish(command);

        command.data = target_speed[1] / 0.035;
        pub_LF.publish(command);
        command.data = target_theta[1];
        pub_LF_angle.publish(command);

        command.data = target_speed[2] / 0.035;
        pub_LB.publish(command);
        command.data = target_theta[2];
        pub_LB_angle.publish(command);

        command.data = target_speed[3] / 0.035;
        pub_RB.publish(command);
        command.data = target_theta[3];
        pub_RB_angle.publish(command);
    }
    else{
        std_msgs::Float32MultiArray floatarray;
        floatarray.data.resize(2);
        floatarray.data[0] = (float)target_speed[0];
        floatarray.data[1] = (float)target_theta[0];
        pub_RF.publish(floatarray);

        floatarray.data[0] = (float)target_speed[1];
        floatarray.data[1] = (float)target_theta[1];
        pub_LF.publish(floatarray);

        floatarray.data[0] = (float)target_speed[2];
        floatarray.data[1] = (float)target_theta[2];
        pub_LB.publish(floatarray);

        floatarray.data[0] = (float)target_speed[3];
        floatarray.data[1] = (float)target_theta[3];
        pub_RB.publish(floatarray);
    }
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
