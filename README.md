# lqr_ros

This is a ROS Package for LQR steering control.
Use Huanyu Smartcar to realize loop tracking.

To compile, jsonCpp, OpenCV and Eigen is required.

To execute, 
Run "rosrun my_lqr my_lqr_node" to test function by @SCUxuhaoran
Run "rosrun my_lqr original_lqr" to test original function
You may download public algorithm through:
"https://github.com/onlytailei/CppRobotics" for Cpp version robotics
"https://github.com/AtsushiSakai/PythonRobotics" for python version

In this package, 
I use json file to save configs. You can change them in /config dir.
I use rostopic /robot_pose_ekf to provide position, so it is necessary to run on real car. To simulate, you'll have to use odometer only, for ekf-position won't change because of AMCL localization.
To switch between /robot_pose_ekf and /odom, modify these code:
lqr_steering_control.cpp::197 to line 198
lqr_steering_control.cpp::210 data type "const nav_msgs::Odometry::ConstPtr& odom" to "geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odom"
as well as header file:
lqr_node.h::41 "void odomCallback(const nav_msgs::Odometry::ConstPtr& odom)" to "void odomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odom)"
