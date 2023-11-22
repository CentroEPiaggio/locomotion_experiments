import os
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    node=Node(
        package = 'locomotion_experiments',
        name = 'cmd_vel_node',
        executable = 'cmd_vel_node',
        parameters = [{'publication_rate': 200},
                      {'duration': 5.0},
                      {'start_delay': 1.0}]
    )


    policy = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare("rlg_quad_controller"), "launch", "mulinex_simulation.launch.py"])]
            ),
    )


    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a'],
            output='screen'
        ),
        node,
        policy,
    ])