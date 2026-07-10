#!/bin/bash

source /opt/ros/jazzy/setup.bash
source ~/Documents/Nexus_Gazebo/install/setup.bash

echo "Starting Gazebo..."
gz sim -r nexus.urdf &
sleep 3

# echo "Starting ROS2-Gazebo bridge..."
# ros2 run ros_gz_bridge parameter_bridge \
#   /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
#   /model/vehicle_blue/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry &
# sleep 2

echo "Starting ROS2-Gazebo bridge..."
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  /ultrasonic/front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /ultrasonic/front_right@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /ultrasonic/front_left@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /ultrasonic/right@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /ultrasonic/left@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /ultrasonic/back@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan &
sleep 2

echo "Starting all"
ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image
ros2 run ros_gz_bridge parameter_bridge \
  /imu@sensor_msgs/msg/Imu@gz.msgs.IMU
ros2 run nexus_gazebo cam
ros2 run nexus_gazebo cam_localization
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
# echo "Starting robot_state_publisher..."
# ros2 run robot_state_publisher robot_state_publisher nexus.urdf &
# sleep 1

# echo "Starting robot control GUI..."
# python3 scripts/robot_control_gui.py

echo "Starting wall follower..."
ros2 run nexus_gazebo obstacle_avoidance