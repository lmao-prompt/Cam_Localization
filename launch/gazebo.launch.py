import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    sdf_path = os.path.join(os.path.dirname(__file__), '..', 'two_wheel_robot.sdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', os.path.join('/usr/share/gz/gz-sim8/worlds', 'empty.sdf')],
            output='screen',
        ),
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'ros_gz_sim', 'create',
                         '-file', sdf_path,
                         '-name', 'two_wheel_robot',
                         '-x', '0', '-y', '0', '-z', '0.10'],
                    output='screen'
                ),
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    arguments=[sdf_path]
                ),
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                        '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                        '/model/two_wheel_robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    ],
                    output='screen'
                ),
            ]
        ),
    ])
