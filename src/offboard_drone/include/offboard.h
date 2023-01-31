#ifndef OFFBOARD_H
#define OFFBOARD_H

#include <ros/ros.h>
#include "subUtil.h"

#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <nav_msgs/Path.h>

class Offboard
{

public:
    Offboard(ros::NodeHandle nh);
    ~Offboard();

    void init();
    void ready();
    void setPath();
    void addWayPoint(int wp_num);
    void wpFlight();  
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void curPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);

private:
    ros::NodeHandle nh;

    int wp_idx;
    float cmd_x, cmd_y, cmd_z;
    
    float x_err, y_err, z_err;
    float dist;

    float cur_pos[3];

    float wp_x[100];
    float wp_y[100];
    float wp_z[100];
    
    geometry_msgs::PoseStamped pos;
    geometry_msgs::TwistStamped vel;
    mavros_msgs::State current_state;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;

    nav_msgs::Path path;

    ros::Subscriber state_sub;
    ros::Subscriber cur_pos_sub;
    ros::Subscriber path_sub;

    ros::Publisher cmd_pos_pub;
    ros::Publisher cmd_vel_pub;
    ros::Publisher cmd_path_pub;

    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    
    ros::Time last_request;
};


#endif