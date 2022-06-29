from matplotlib.pyplot import get
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
import argparse
from .tb4_ride import TurtleRide


 
class TurtleGoal(Node):

    def __init__(self):
        """
        Turtle goal constructor to initialize nodes, subscribers, publishers and parameters
        """
        super().__init__('turtlebot4_visual_nav_goal')
        self.declare_parameter('goal_pose', [5.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.publisher_ = self.create_publisher(Pose,'/goal', 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.publish_goal)
        self.i = 0

    def publish_goal(self):
        
        if(True): #[TODO] condition for robot reach state
            get_goal_pose = self.get_parameter('goal_pose')
            goal_pose_list = str(get_goal_pose.value).strip('][').split(', ')
            goal_pose = Pose()
            goal_pose.position.x = float(goal_pose_list[0])
            goal_pose.position.y = float(goal_pose_list[1])
            goal_pose.position.z = float(goal_pose_list[2])
            goal_pose.orientation.x = float(goal_pose_list[3])
            goal_pose.orientation.y = float(goal_pose_list[4])
            goal_pose.orientation.z = float(goal_pose_list[5])
            goal_pose.orientation.w = float(goal_pose_list[6])

            self.publisher_.publish(goal_pose)
            #self.get_logger().info('Publishing Goal Node')
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
         