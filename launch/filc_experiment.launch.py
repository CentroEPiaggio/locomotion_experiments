######################################## MULINEX

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
import numpy as np
import yaml

def generate_launch_description():

    filc_cont_params = os.path.join(
        get_package_share_directory('filc_controller'),
        'config',
        'mulinex_real_robot_filc_cont_config.yaml'
        )

    filc_cont_controller = Node(
        package = 'filc_controller',
        name = 'filc_controller_continuous_node',
        executable = 'filc_controller_continuous_node',
        output='screen',
        parameters = [filc_cont_params]
    )


    return LaunchDescription([
        ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
        output='screen'
        ),
        filc_cont_controller,

    
  ])
