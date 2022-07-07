
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




ARGUMENTS = [

    DeclareLaunchArgument('world', default_value='custom_world',
                          description='Ignition World')
]


def generate_launch_description():

    # Directories
    pkg_unit01_simulation_L2_gazebo_world_bringup = get_package_share_directory(
        'unit01_simulation_L2_gazebo_world')


    pkg_ros_ign_gazebo = get_package_share_directory(
        'ros_ign_gazebo')

    # Set ignition resource path
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_unit01_simulation_L2_gazebo_world_bringup, 'worlds')])


    # Paths
    ign_gazebo_launch = PathJoinSubstitution(
        [pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])


    # Ignition gazebo
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        launch_arguments=[
            ('ign_args', [
                LaunchConfiguration('world'), '.sdf',
                ' -v 4',
                ' --gui-config '])
        ]
    )


  
    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ign_resource_path)
    ld.add_action(ignition_gazebo)
  
    return ld
