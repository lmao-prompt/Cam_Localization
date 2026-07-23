#!/bin/bash

source /opt/ros/jazzy/setup.bash
source ~/Downloads/Nexus_Gazebo/install/setup.bash

echo "Starting Gazebo..."
gz sim -r nexus.urdf &
sleep 5

echo "Starting ROS-Gazebo bridge..."
ros2 run ros_gz_bridge parameter_bridge \
    /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
    /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
    /imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
    /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image &
sleep 1

# ros2 run ros_gz_bridge parameter_bridge \
#     /world/car_world/create@ros_gz_interfaces/srv/SpawnEntity \
#     /world/car_world/remove@ros_gz_interfaces/srv/DeleteEntity &

# ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

echo "Starting nodes..."
ros2 run nexus_gazebo aruco &
ros2 run nexus_gazebo cam_localization &
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ros2 run image_transport republish raw compressed \
    --ros-args \
    -r in:=/camera/image_raw \
    -r out/compressed:=/camera/image_raw/compressed

wait