import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,RegisterEventHandler, LogInfo, EmitEvent
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
## primo video è n 19 (fermo)


def generate_launch_description():
        csv = LaunchConfiguration('csv', default='locomotion_experiments_data.csv')
        csv_declare = DeclareLaunchArgument(
                'csv',
                default_value='locomotion_experiments_data.csv',
                description='csv file to save info, including extension. Default: locomotion_experiments_data.csv'
                )

        exp = LaunchConfiguration('exp', default='22_04')
        exp_declare = DeclareLaunchArgument(
                'exp',
                default_value='experiment',
                description='experiment name'
                )

        use_joy = LaunchConfiguration('use_joy', default='False')
        use_joy_declare = DeclareLaunchArgument(
                'use_joy',
                default_value='False',
                description='Use joystick and disable cmd_vel_node'
                )
        
        vel = LaunchConfiguration('vel', default=-0.8)
        vel_declare = DeclareLaunchArgument(
                'vel',
                default_value='22_04',
                description='velocity'
                )
        
                
        duration = LaunchConfiguration('duration', default=2.0)
        duration_declare = DeclareLaunchArgument(
                'duration',
                default_value='22_04',
                description='duration'
                )
        
        
        cmdvel_node=Node(
                package = 'locomotion_experiments',
                name = 'cmd_vel_node',
                executable = 'cmd_vel_node',
                parameters =    [{'publication_rate': 200},
                                {'duration': duration},
                                {'start_delay': 6.0},
                                {'top_v': vel}],
                condition=IfCondition(
                        PythonExpression([
                               'not ',
                                use_joy
                        ])
                )
        )

        steering_joy = Node(
                package = 'rqt_robot_steering',
                name = 'rqt_robot_steering',
                executable = 'rqt_robot_steering',
                condition=IfCondition(
                        PythonExpression([
                                use_joy
                        ])
                )
        )

        policy = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        [PathJoinSubstitution([FindPackageShare("rlg_quad_controller"), "launch", "mulinex_inference.launch.py"])]
                ),
        )

        save_csv_process = ExecuteProcess(
                cmd=[
                        'echo', exp, use_joy, vel, duration, '>>', csv
                ]

        )
        bag_process = ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-a', '-o', exp, '-s', 'mcap'],
                output='screen'
        )

        # TODO: stops bag recording and policy node when cmd_vel_node is done
        shutdown_event = RegisterEventHandler(
                event_handler= OnProcessExit(
                target_action=bag_process,
                on_exit=[
                LogInfo(
                        msg="BAG NODE CRASHED. STOPPING EXPERIMENT."),
                EmitEvent(
                        event=Shutdown())]))
        
        return LaunchDescription([
                exp_declare, 
                use_joy_declare,
                duration_declare,
                vel_declare,
                csv_declare,
                save_csv_process,
                steering_joy,
                bag_process,
                cmdvel_node,
                policy,
                shutdown_event,

        ])