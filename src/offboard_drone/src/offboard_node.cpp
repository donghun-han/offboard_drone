#include <ros/ros.h>
#include "offboard.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;

    Offboard offboard(nh);

    offboard.ready();

    ros::Rate rate(20.0);
    while(ros::ok())
    {
        offboard.wpFlight();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}