# 2WD Differential Drive Simulation

[![ROS 2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/index.html)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)](https://gazebosim.org/home)

## How to

```bash
# 1. Make folder
mkdir -p ~/diff_ws/src
cd ~/diff_ws/src

# 2. copy this
git clone https://github.com/lmao-prompt/Cam_Localization.git

# 3. build
cd ~/diff_ws
colcon build --symlink-install
source install/setup.bash

# 4. run
./start.sh
ros2 run ros_gz_bridge parameter_bridge \
  /imu@sensor_msgs/msg/Imu@gz.msgs.IMU
ros2 run nexus_gazebo cam
ros2 run nexus_gazebo cam_localization
ros2 run nexus_gazebo movement
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
