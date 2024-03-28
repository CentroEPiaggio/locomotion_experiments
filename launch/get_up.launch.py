import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

## primo video è n 19 (fermo)


def generate_launch_description():

        getup_node=Node(
                package = 'locomotion_experiments',
                name = 'getup',
                executable = 'getup_node',
        )

        return LaunchDescription([

                getup_node
                
        ])