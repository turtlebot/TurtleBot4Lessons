import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription, SomeSubstitutionsType, Substitution
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


ARGUMENTS = [DeclareLaunchArgument('world', default_value='bug2_world',
                          description='Ignition World')]


def generate_launch_description():
    
    # Directories
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')
    
    # Paths
    turtlebot4_ros_ignition_launch = PathJoinSubstitution(
    [pkg_turtlebot4_ignition_bringup, 'launch', 'ignition.launch.py'])

    # ROS world config
    turtlebot4_ros_ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot4_ros_ignition_launch]),
        launch_arguments=[('world', LaunchConfiguration('world'))]
    )

    # extented_kalman_filter_node = Node(
    #         package='ros2_tb4_visual_nav',
    #         namespace='ekf',
    #         executable='ekf',
    #         name='extented_kalman_filter'
    #     )

    # tb4_planner = Node(
    #         package='ros2_tb4_visual_nav',
    #         namespace='tb4_rider',
    #         executable='tb4_rider',
    #         name='turtlebot4_ride'
    #     )
    
    # goal_pose = Node(
    #         package='ros2_tb4_visual_nav',
    #         namespace='publish_goal',
    #         executable='tb4_goal',
    #         name='tb4_goal_pose'
    #     )

    # Define LaunchDescription variable
    launch_desc = LaunchDescription(ARGUMENTS)
    launch_desc.add_action(turtlebot4_ros_ignition)
    # launch_desc.add_action(extented_kalman_filter_node)
    # launch_desc.add_action(goal_pose)
    # launch_desc.add_action(tb4_planner)
    return launch_desc