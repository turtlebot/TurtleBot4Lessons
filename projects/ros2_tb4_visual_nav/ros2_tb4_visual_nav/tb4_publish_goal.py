import rclpy 
from time import sleep 
from rclpy.node import Node
from std_msgs.msg import String 
from geometry_msgs.msg import Twist            
from sensor_msgs.msg import LaserScan    
from geometry_msgs.msg import Pose 
from std_msgs.msg import Float64MultiArray
from rclpy.qos import qos_profile_sensor_data 
import numpy as np 
import math 
from .tb4_ride import TurtleRide


 
class TurtleGoal(Node):

    def __init__(self):
        """
        Turtle goal constructor to initialize nodes, subscribers, publishers and parameters
        """
        super().__init__('turtlebot4_visual_nav_goal')

        self.publisher_ = self.create_publisher(Pose,'/goal', 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.publish_goal)
        self.i = 0

    def publish_goal(self):
        
        if(True): #[TODO] condition for robot reach state
            goal_pose = Pose()
            goal_pose.position.x = 0.0
            goal_pose.position.y = 0.0
            goal_pose.position.z = 0.0
            goal_pose.orientation.x = 0.0
            goal_pose.orientation.y = 0.0
            goal_pose.orientation.z = 0.0
            goal_pose.orientation.w = 0.0

            self.publisher_.publish(goal_pose)
            self.get_logger().info('Publishing Goal Node')
            self.i += 1
        else:
            self.get_logger().info('Turtle bot reached destination')

        

def main(args=None):
    
    rclpy.init(args=args)
    turtlebot_4_goal = TurtleGoal()
    rclpy.spin(turtlebot_4_goal)
    turtlebot_4_goal.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
         