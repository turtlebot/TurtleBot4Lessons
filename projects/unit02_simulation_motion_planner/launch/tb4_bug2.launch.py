from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

ARGUMENTS = [DeclareLaunchArgument('my_goal_pose', default_value='[8.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
                          description='Goal position in X and Y coordinates')]

def generate_launch_description():
    my_goal_pose = LaunchConfiguration('my_goal_pose'),
    planner = Node(
        package='unit02_simulation_motion_planner',
        namespace='tb4_rider',
        executable='tb4_rider',
        name='turtlebot4_ride'
    )
    estimator = Node(
        package='unit02_simulation_motion_planner',
        namespace='ekf',
        executable='ekf',
        name='extented_kalman_filter'
    )
    goal_pub = Node(
        package='unit02_simulation_motion_planner',
        namespace='publish_goal',
        executable='tb4_goal',
        parameters=[
        {'my_goal_pose': LaunchConfiguration('my_goal_pose')}],
        name='tb4_goal_pose'
    )
    # Define LaunchDescription variable
    launch_desc = LaunchDescription(ARGUMENTS)
    launch_desc.add_action(planner)
    launch_desc.add_action(estimator)
    launch_desc.add_action(goal_pub)

    return launch_desc