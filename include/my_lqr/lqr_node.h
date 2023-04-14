#ifndef LQR_NODE_H
#define LQR_NODE_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <thread>
#include <iomanip>
#include "ros/ros.h"
#include <iostream>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "cubic_spline.h"
#include "motion_model.h"
#include "cpprobotics_types.h"

namespace cpprobotics{

class LQR_start_object
{
public:
    LQR_start_object();
    ~LQR_start_object();
    int steering_control();
    int special_steering_control_to_start_point();
    void cmd_vel_run(float linear, float angular, int x, int y, int z, int th);
    void update (State& state, float a, float delta);
    void closed_loop_prediction(Vec_f cx, Vec_f cy, Vec_f cyaw, Vec_f ck, Vec_f speed_profile, Poi_f goal);
    void special_closed_loop_prediction_to_start_point(Vec_f cx, Vec_f cy, Vec_f cyaw, Vec_f ck, Vec_f speed_profile, Poi_f goal);

private:
    /** Ros node define*/
    ros::NodeHandle n;
    ros::Time current_time, last_time;
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
    float p_x;
    float p_y;
    float p_z;
    float p_yaw;
    ros::Subscriber odom_sub;
    ros::Publisher cmd_vel_pub;
    tf::TransformBroadcaster odom_broadcaster;
    tf::TransformListener odom_listener;

};

}
#endif
