from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unit02_simulation_motion_planner',
            namespace='tb4_rider',
            executable='tb4_rider',
            name='turtlebot4_ride'
        ),
        Node(
            package='unit02_simulation_motion_planner',
            namespace='ekf',
            executable='ekf',
            name='extented_kalman_filter'
        ),
        Node(
            package='unit02_simulation_motion_planner',
            namespace='publish_goal',
            executable='tb4_goal',
            parameters=[
                {'my_goal_pose': [2.0, 8.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
            ],
            name='tb4_goal_pose'
        )
    ])