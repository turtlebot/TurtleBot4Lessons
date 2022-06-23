import math 
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
 
class TurtleRide(Node):
    """
    Create a TurtleBot 4 Controller class
    """
 
    def __init__(self):
        

        super().__init__('turtlebot4_visual_nav')
         
        # Subscribing to Lidar data     
        self.scan_subscriber = self.create_subscription(LaserScan,'/scan',self.lidar_scan_callback,qos_profile=qos_profile_sensor_data)

        # Subscribing to State estimator
        self.subscription = self.create_subscription(Float64MultiArray,'/state_estimator',self.state_estimate_callback,10)
        self.subscription
     
        # Subsribing to Goal Position
        self.subscription_goal_pose = self.create_subscription(Pose,'/goal',self.pose_estimated,10)
             
        # Publishing command velocity
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
         
        # Initializing Lidar sensor data in zones 
        self.left = 999999.9
        self.leftfront = 999999.9
        self.front = 999999.9 
        self.rightfront = 999999.9 
        self.right = 999999.9
 
        # Setting forward speed of the Turtle bot 4
        self.forward_speed = 0.5
         
        # present pose and orientation of the TB4 in the global reference frame
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.Isbug2 = True
        self.cal_line_distance = False
        self.slope = 0
        self.intercept = 0
        self.xstart = 0
        self.xgoal = 0
        self.ystart = 0
        self.ygoal = 0
  
    def pose_estimated(self,msg):

        self.goal_x = [msg.position.x]
        self.goal_y = [msg.position.y]
        self.goal_max = len(self.goal_x) - 1       
     
    def lidar_scan_callback(self, msg):

        # Zonning lidar readings separated by 45 degrees  
        self.left = msg.ranges[180]
        self.leftfront = msg.ranges[135]
        self.front = msg.ranges[90]
        self.rightfront = msg.ranges[45]
        self.right = msg.ranges[0]
         
        if self.robot_mode == "obstacle avoidance":
            self.avoid_obstacles()
             
    def state_estimate_callback(self, msg):

        # Update gobal pose
        curr_state = msg.data
        self.current_x = curr_state[0]
        self.current_y = curr_state[1]
        self.current_yaw = curr_state[2]
         
        # estimating goal positions
        if self.goal_x == False and self.goal_y == False:
            return
                 
        # Call bug 2
             
         
    def avoid_obstacles(self):
        #logic to avoid obstacles
        #TODO
        return
                             
    def path_planner(self):

        # Creating a Twist message
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
         

             
    def boundary_check(self):
        #TODO
        return
   
         
    def bug2_algorithm(self):
     
        # Calculating distance between goal and source iteratively 
        if self.cal_line_distance == False:

            self.robot_mode = "goal_mode"            
 
            self.x_start = self.current_x
            self.x_goal = self.goal_x[self.goal_idx]
            self.y_start = self.current_y
            self.y_goal = self.goal_y[self.goal_idx]
             
            # Calculating Slope of the line from start to goal
            self.slope = ((self.y_goal - self.y_start) / (self.x_goal - self.x_start))
             
            # Estimating intercept
            self.intercept = self.y_goal - (
                    self.slope * self.x_goal) 
                 
            self.cal_line_distance = True
             
        if self.robot_mode == "goal_mode":
            self.path_planner()           
        elif self.robot_mode == "wall_mode":
            self.boundary_check()

def main(args=None):
 
    rclpy.init(args=args)
    turtlebot_4 = TurtleRide()
    rclpy.spin(turtlebot_4)
    turtlebot_4.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()