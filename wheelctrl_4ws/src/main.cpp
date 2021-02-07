#include <ros/ros.h>
#include "cmdvel24ws.h"

std::string node_name = "wheelctrl_4ws";

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    /**
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    **/

    int loop_rate = 30;
    float body_width = 0.440;
    int lost_time_threshold = 500;
    pnh.getParam("control_frequency", loop_rate);
    pnh.getParam("lost_time_threshold", lost_time_threshold);
    pnh.getParam("body_width", body_width);

    VelConverter converter(nh, body_width, lost_time_threshold, loop_rate);
    return 0;
}
