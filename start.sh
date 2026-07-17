#!/bin/bash

source /opt/ros/jazzy/setup.bash
source ~/Downloads/Nexus_Gazebo/install/setup.bash

echo "Starting Gazebo..."
gz sim -r nexus.urdf &
sleep 5

echo "Starting ROS2-Gazebo bridge..."
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  /imu@sensor_msgs/msg/Imu[gz.msgs.IMU &
sleep 3

echo "Starting script"
ros2 run nexus_gazebo aruco &
ros2 run nexus_gazebo cam_localization &
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ros2 run image_transport republish raw compressed   \
  --ros-args -r in:=/camera/image_raw -r out/compressed:=/camera/image_raw/compressed 
# socat TCP-LISTEN:9090,fork,reuseaddr TCP:100.98.131.125:9090
sleep 5
