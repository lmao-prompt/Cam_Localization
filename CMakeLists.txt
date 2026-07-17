cmake_minimum_required(VERSION 3.8)
project(nexus_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(waypoint src/waypoint.cpp)
ament_target_dependencies(waypoint
rclcpp geometry_msgs tf2)

add_executable(movement src/movement.cpp)
ament_target_dependencies(movement
rclcpp geometry_msgs tf2)

add_executable(cam_localization src/cam_localization.cpp)
ament_target_dependencies(cam_localization
rclcpp geometry_msgs sensor_msgs tf2)

add_executable(aruco_localization src/aruco_localization.cpp)
ament_target_dependencies(aruco_localization
rclcpp geometry_msgs sensor_msgs tf2 cv_bridge OpenCV)
target_link_libraries(aruco_localization nlohmann_json::nlohmann_json)

add_executable(cam src/cam.cpp)
ament_target_dependencies(cam
rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV)

add_executable(aruco src/aruco.cpp)
ament_target_dependencies(aruco
rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV)

add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
ament_target_dependencies(obstacle_avoidance rclcpp sensor_msgs geometry_msgs)

install(TARGETS cam DESTINATION lib/${PROJECT_NAME})
install(TARGETS cam_localization DESTINATION lib/${PROJECT_NAME})
install(TARGETS obstacle_avoidance DESTINATION lib/${PROJECT_NAME})
install(TARGETS waypoint DESTINATION lib/${PROJECT_NAME})
install(TARGETS movement DESTINATION lib/${PROJECT_NAME})
install(TARGETS aruco DESTINATION lib/${PROJECT_NAME})
install(TARGETS aruco_localization DESTINATION lib/${PROJECT_NAME})

ament_package()