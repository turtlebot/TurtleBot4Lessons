from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_tb4_visual_nav',
            namespace='tb4_perception',
            executable='tb4_perception',
            name='turtlebot4_ride_perception'
        )
    ])