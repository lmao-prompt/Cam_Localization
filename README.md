# 2WD Differential Drive Simulation

[![ROS 2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/index.html)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)](https://gazebosim.org/home)

## How to

```bash
# 1. copy this
git clone https://github.com/lmao-prompt/Cam_Localization.git

# 2. build
cd Cam_Localization
colcon build --symlink-install
source install/setup.bash

#3 calibrate
make sure to calibrate hsv object and map, then adjust aruco or apriltag 

# 4. run
1. start micro ros first(still have issue on this file but u can use separate folder to use that)
2. start testaruco.py
3. start aruco_localization
```

#Everything should be work i guess
