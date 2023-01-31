#include "offboard.h"


Offboard::Offboard(ros::NodeHandle nh):nh(nh)
{
    wp_idx = 0;
    init();
    setPath();

}

Offboard::~Offboard()
{

}

void Offboard::init(){
    state_sub = nh.subscribe("mavros/state", 10, &Offboard::stateCallback, this);
    cur_pos_sub = nh.subscribe("mavros/local_position/pose", 10, &Offboard::curPosCallback, this);
    path_sub = nh.subscribe("path_cmd", 10, &Offboard::pathCallback, this);

    cmd_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    cmd_path_pub = nh.advertise<nav_msgs::Path>("path_cmd", 10);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

}

void Offboard::ready(){
    ros::Rate rate(20.0);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

/*     mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true; */

    geometry_msgs::PoseStamped setpoint;
    setpoint.pose.position.x = 0.0;
    setpoint.pose.position.y = 0.0;
    setpoint.pose.position.z = 0.0;
    setpoint.pose.orientation.z = 0.0;
    
    for(int i = 100; ros::ok() && i > 0; --i){
        cmd_pos_pub.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }
    offb_set_mode.request.custom_mode = "OFFBOARD";

    arm_cmd.request.value = true;
    
    last_request = ros::Time::now();
}

void Offboard::setPath(){
    addWayPoint(10);

}

void Offboard::addWayPoint(int wp_num){
    geometry_msgs::PoseStamped wp;

    for(int i = 0; i < wp_num; i++){
        wp.header.stamp = ros::Time::now();

        wp.pose.position.x = 3*sin(PI/2*i);
        wp.pose.position.y = 3*sin(PI/2*i);
        wp.pose.position.z = 3 + 2*cos(PI/2*i);
        wp.pose.orientation.z = 1.0;

        path.poses.push_back(wp);

        ROS_INFO("Add waypoint : x = %f, y = %f, z = %f\n", path.poses[i].pose.position.x, path.poses[i].pose.position.y, path.poses[i].pose.position.z);
        ROS_INFO("# of waypoint: %d\n", i+1);

    }
    cmd_path_pub.publish(path);
}

void Offboard::wpFlight(){


    if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0))){
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 
	    else if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0))){
            if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
        
	    else{
            if(wp_idx < path.poses.size()){

            }     
            x_err = cur_pos[0] - wp_x[wp_idx];
            y_err = cur_pos[1] - wp_y[wp_idx];
            z_err = cur_pos[2] - wp_z[wp_idx];


            dist = sqrt(x_err*x_err + y_err*y_err + z_err*z_err);
            //ROS_INFO("wp_idx: %d, dist : %f", wp_idx, dist);

            if (dist > 0.1){
                cmd_x = satmax(Kpx*(wp_x[wp_idx] - cur_pos[0]), 0.5);
                cmd_y = satmax(Kpx*(wp_y[wp_idx] - cur_pos[1]), 0.5);
                cmd_z = satmax(Kpz*(wp_z[wp_idx] - cur_pos[2]), 0.5);

                vel.twist.linear.x = cmd_x;
                vel.twist.linear.y = cmd_y;
                vel.twist.linear.z = cmd_z;

                cmd_vel_pub.publish(vel); 

            }
            
            else{
                ROS_INFO("waypoint %d reached : x = %f, y = %f, z = %f\n", wp_idx+1, wp_x[wp_idx], wp_y[wp_idx], wp_z[wp_idx]);
                wp_idx = wp_idx + 1;

            }
                
        }    
    
}

void Offboard::stateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void Offboard::curPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pos = *msg;

    cur_pos[0] = pos.pose.position.x;
    cur_pos[1] = pos.pose.position.y;
    cur_pos[2] = pos.pose.position.z;
}
void Offboard::pathCallback(const nav_msgs::Path::ConstPtr& msg){
    path = *msg;
    for(int idx = 0; idx < path.poses.size(); idx++){
    // for(int idx = 0; idx < sizeof(wp_x)/sizeof(float); idx++){
        wp_x[idx] = path.poses[idx].pose.position.x;
        wp_y[idx] = path.poses[idx].pose.position.y;
        wp_z[idx] = path.poses[idx].pose.position.z;

        ROS_INFO("path cb count %d", idx+1);
        ROS_INFO("<callback> wp %d : x = %f, y = %f, z = %f\n", idx+1, wp_x[idx], wp_y[idx], wp_z[idx]);

    }
}
