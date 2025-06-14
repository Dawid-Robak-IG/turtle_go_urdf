import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration # <--- ADD THIS LINE

def generate_launch_description():
    # Define package share directory for cleaner paths
    pkg_share_dir = FindPackageShare('turtle_go_urdf')

    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            pkg_share_dir,
            'urdf',
            'robot.urdf.xacro'
        ])
    ])

    # Declare a launch argument for the controller parameters file
    declare_controller_params_file_cmd = DeclareLaunchArgument(
        'controller_params_file',
        default_value=PathJoinSubstitution([
            pkg_share_dir,
            'config',
            'diff_drive_controller.yaml'
        ]),
        description='Path to the controller parameter file'
    )

    # Access the LaunchConfiguration for the controller_params_file
    controller_params = LaunchConfiguration('controller_params_file')

    return LaunchDescription([
        declare_controller_params_file_cmd, # Declare the argument first

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                controller_params # This should now correctly resolve to the path
            ],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', '--ros-args', '-r', '__node:=diff_drive_controller_spawner'],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'], 
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),

        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            prefix='xterm -e',
            remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel')],
            output='screen'
        )
    ])