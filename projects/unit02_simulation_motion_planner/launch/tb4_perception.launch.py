from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unit02_simulation_motion_planner',
            namespace='tb4_perception',
            executable='tb4_perception',
            name='turtlebot4_ride_perception'
        )
    ])