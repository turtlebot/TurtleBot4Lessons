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

#[TODO] Undocking before planning 
# To know if turtle bot has reached goal
isReached = False

class TurtleRide(Node):

    def __init__(self):
        """
        TurtleRide constructor to initialize nodes, subscribers, publishers and parameters
        """
        super().__init__('turtlebot4_visual_nav')
         
        # Subscribing to Lidar data     
        self.scan_subscriber = self.create_subscription(LaserScan,'/scan',self.lidar_scan,qos_profile=qos_profile_sensor_data)

        # Subscribing to State estimator
        self.subscription = self.create_subscription(Float64MultiArray,'/state_estimator',self.state_estimation,10)
        self.subscription 
        
        # Subsribing to Goal Position
        self.subscription_goal = self.create_subscription(Pose,'/goal',self.goal_estimation,10)
             
        # Publishing command velocity
        self.publisher_ = self.create_publisher(Twist,'/cmd_vel', 10)
         
        # Various types of Robot modes
        ### "avoid_mode": To avoid obstacles
        ### "goal_mode": To navigate to x,y coordinate   
        ### "wall_mode": to follow a wall 

        self.tb4_mode = "goal_mode"

        # Possible goal states for turtlebot 4
        ### "orient_to_goal": change orientation towards a goal (x, y)
        ### "straight_to_goal": apply linear velocity towards goal (x, y)
        ### "goal_state": Final goal reached
        
        self.tb4_goal_state = "orient_to_goal"

        # Possible states for Turtlebot 4 when it reaches wall
        ### "turn": By default tb4 turns towards left
        ### "identify_wall": tb4 attempts to identify the wall       
        ### "along_wall": tb4 moves algon to the wall with certain threshold
        
        self.tb4_wall_state = "turn"

        # Goal Position
        self.goal_x = False 
        self.goal_y = False

        # Initializing Lidar sensor data in zones 

        self.left = 9999999999.9
        self.leftfront = 9999999999.9 
        self.front = 9999999999.9 
        self.rightfront = 9999999999.9
        self.right = 9999999999.9 
 
        # Essential Parameters

        # Linear velocity of TB4
        self.linear_velocity = 0.75 

        # Angular velocity of TB4
        self.angular_velocity = 0.3

        # Angular turn velocity of TB4
        self.angular_turn_velocity = 0.09
         
        # live position and orientation of the robot in the world frame
        self.x_pose_live = 0.0
        self.y_pose_live = 0.0
        self.orientation_live = 0.0

        # Proximity between TB4 and an obstacle in meters
        self.collision_avoidance_proximity = 0.3

        # List of all intermediate goal coordinates
        self.goal_id_lst = 0
         
        # tracking end goal coordinates
        self.final_goal_id =  None 
         
        # angular precision +/- 3.0 degrees
        self.angle_threshold = 3.0 * (math.pi / 180) 

        # TB4 to goal distance
        self.tb4_goal_dist = 0.1
          
        self.sharp_turn = 1.1 
        self.smooth_turn = 0.1
         
        # minimum wall distance tb4 can walkthrough
        self.tb4_wall_distance = 0.5  
         
        # Collision avoidance theshold with wall
        self.wall_collision_avoidance = 0.2 
    
        self.isBug2 = True
         
        # Flag to calculate slope line between start and goal
        self.is_SG_line_cal = False
         
        # Start-Goal Line Parameters
        self.sg_line_slope = 0
        self.sg_line_y_intercept = 0
        self.sg_line_xstart = 0
        self.sg_line_xgoal = 0
        self.sg_line_ystart = 0
        self.sg_line_ygoal = 0
 
        # Point at which Turtle bot 4 reahces the wall and follows the wall 
        # reach(x, y)
        self.reach_x = 0
        self.reach_y = 0
         
        # Point at which Turtle bot 4 leaves the wall
        # Then orient & follow towards goal      
        self.leave_x = 0
        self.leave_y = 0

        # Distance between the hit point and the goal in meters
        self.dist_reach_to_goal = 0.0
         
        # Distance between the leave point and the goal 
        self.dist_leave_to_goal = 0.0
         
        # minimum required distance between reach and leave 
        self.dist_leave_to_reach = 0.25

        
         
    def goal_estimation(self,msg):
        
        self.goal_x = [msg.position.x]
        self.goal_y = [msg.position.y]
        self.final_goal_id = len(self.goal_x) - 1       
     
    def lidar_scan(self, msg):
        """
        LaserScan callback method
        """

        self.left = msg.ranges[180]
        self.leftfront = msg.ranges[135]
        self.front = msg.ranges[90]
        self.rightfront = msg.ranges[45]
        self.right = msg.ranges[0]

        if self.tb4_mode == "avoid_mode":
            self.obstacle_avoidance()
             
    def state_estimation(self, msg):
        """
        Method to estimate the position and orientation
        """
        # Storing current pose and orientation as current state
        current_state = msg.data
        self.x_pose_live = current_state[0]
        self.y_pose_live = current_state[1]
        self.orientation_live = current_state[2]
         
        # Condition to receive goal coordinates from /goal publisher 
        if self.goal_x == False and self.goal_y == False:
            return
  
        # Run bug2_algorithm Motion Planner
        if self.isBug2 == True:
            self.bug2_algorithm()
           
         
    def obstacle_avoidance(self):
        """
        Method to avoid static obstacles from lidar data
        """
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
         

        distance = self.collision_avoidance_proximity
        if   self.leftfront > distance and self.front > distance and self.rightfront > distance:
            msg.linear.x = self.linear_velocity # forward
        elif self.leftfront > distance and self.front < distance and self.rightfront > distance:
            msg.angular.z = self.angular_velocity  # turn left
        elif self.leftfront > distance and self.front > distance and self.rightfront < distance:
            msg.angular.z = self.angular_velocity  
        elif self.leftfront < distance and self.front > distance and self.rightfront > distance:
            msg.angular.z = -self.angular_velocity # turn right
        elif self.leftfront > distance and self.front < distance and self.rightfront < distance:
            msg.angular.z = self.angular_velocity 
        elif self.leftfront < distance and self.front < distance and self.rightfront > distance:
            msg.angular.z = -self.angular_velocity 
        elif self.leftfront < distance and self.front < distance and self.rightfront < distance:
            msg.angular.z = self.angular_velocity 
        elif self.leftfront < distance and self.front > distance and self.rightfront < distance:
            msg.linear.x = self.linear_velocity
        else:
            pass
             
        # publish to /cmd_vel
        self.publisher_.publish(msg)
                             
    def path_planner(self):
        """
        Method to plan path towards goal position
        """
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        
        if self.isBug2 == True:
         
            # Cheking for wall 
            distance = 0.2
            if (self.leftfront < distance or self.front < distance or self.rightfront < distance):
             
                # If there is wall then change the tb4 mode
                self.tb4_mode = "wall_mode"
                 
                # Point at which Turtle bot 4 reaches the wall and follows the wall 
                self.reach_x = self.x_pose_live
                self.reach_y = self.y_pose_live
                 
                # Find the distance from reach point to goal point
                self.dist_reach_to_goal = (math.sqrt((pow(self.goal_x[self.goal_id_lst] - self.reach_x, 2)) + (pow(self.goal_y[self.goal_id_lst] - self.reach_y, 2))))    
                     
                # Perform a sharp turn to follow the wall
                msg.angular.z = self.sharp_turn
                         
                # publish new msg to command velocity
                self.publisher_.publish(msg)
                 
                # Exit this function        
                return
             
        # Turn the orientation of the tb4 accordingly to reach the goal pose      
        if (self.tb4_goal_state == "orient_to_goal"):
             
            # calculate the estimate yaw angle depending on the current pose and goal pose
            desired_yaw = math.atan2(
                    self.goal_y[self.goal_id_lst] - self.y_pose_live,
                    self.goal_x[self.goal_id_lst] - self.x_pose_live)
                     
            yaw_error = desired_yaw - self.orientation_live
             
            # change the orientation if error is greater than anticipated threshold
            if math.fabs(yaw_error) > self.angle_threshold:
             
                if yaw_error > 0:    
                    # turn left     
                    msg.angular.z = self.angular_turn_velocity               
                else:
                    # Turn right 
                    msg.angular.z = -self.angular_turn_velocity
                 
                # Publish command velocity acoordingly
                self.publisher_.publish(msg)
                 
            # If the orentation is good change the goal state to straight
            else:               
                self.tb4_goal_state = "straight_to_goal"
                 
                # Publish command velocity acoordingly
                self.publisher_.publish(msg)        
 
        # straight_to_goal                                       
        elif (self.tb4_goal_state == "straight_to_goal"):
             
            pos_error = math.sqrt(pow(self.goal_x[self.goal_id_lst] - self.x_pose_live, 2)+ pow(self.goal_y[self.goal_id_lst] - self.y_pose_live, 2)) 
                         
             
            # estimate distance between goal and current location then repeat the process                       
            if pos_error > self.tb4_goal_dist:
 
                # Apply linear velocity
                msg.linear.x = self.linear_velocity
                     
                # Publish command velocity acoordingly to move the robot
                self.publisher_.publish(msg)
             
                # check if orientation is good        
                desired_yaw = math.atan2(
                    self.goal_y[self.goal_id_lst] - self.y_pose_live,
                    self.goal_x[self.goal_id_lst] - self.x_pose_live)
                 
                # Adjusting the orientation  
                yaw_error = desired_yaw - self.orientation_live      
         
                # estimating if orientation is reasonable and if not chnaging the state of the turtle bot
                if math.fabs(yaw_error) > self.angle_threshold:
                     
                    # Changing the state of the tb4 to adjust orentation
                    self.tb4_goal_state = "orient_to_goal"
                 
            # else change the goal state to goal state reached
            else:           
                # Change the state
                self.tb4_goal_state = "goal_state"
                 
                # Publish command velocity with 0 to stop
                self.publisher_.publish(msg)
         
        # goal_state         
        elif (self.tb4_goal_state == "goal_state"):
                 
            self.get_logger().info('goal_state! X:%f Y:%f' % (
                self.goal_x[self.goal_id_lst],
                self.goal_y[self.goal_id_lst]))
             
            # incrementing to next goal state
            self.goal_id_lst = self.goal_id_lst + 1
         
            # reaching final goal
            if (self.goal_id_lst > self.final_goal_id):
                self.get_logger().info('Goal Reached!!! Planning complete')
                while True:
                    pass
 
            # If not
            else: 
                # change the goal state to ajdust orientation
                self.tb4_goal_state = "orient_to_goal"               
 
            self.is_SG_line_cal = False            
         
        else:
            pass
             
    def boundary_check(self):
        """
        This method enables the turtlebot4 to follow the boundary of a wall.
        """
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0        
 
        if self.isBug2 == True:
         
            # Calculating the closest point on sg line
            x_start_goal_line = self.x_pose_live
            y_start_goal_line = (self.sg_line_slope * (x_start_goal_line)) + (self.sg_line_y_intercept)
                         
            # calculating distance between claculated point
            distance_to_start_goal_line = math.sqrt(pow(x_start_goal_line - self.x_pose_live, 2) + pow(y_start_goal_line - self.y_pose_live, 2)) 
                             
            # what if hit the sg line again? on the second collison with sg-line - we consider the tb4 to leaving the line               
            if distance_to_start_goal_line < 0.1:
             
                # storing the current pose of tbr at that point
                self.leave_x = self.x_pose_live
                self.leave_y = self.y_pose_live
 
                # estimating the distnace between goal and current point
                self.dist_leave_to_goal = math.sqrt(pow(self.goal_x[self.goal_id_lst] - self.leave_x, 2)+ pow(self.goal_y[self.goal_id_lst] - self.leave_y, 2)) 
             
                # check for diffrentiating leave point and reach point. if the distance to goal is less then it is reach point
                difference = self.dist_reach_to_goal - self.dist_leave_to_goal
                if difference > self.dist_leave_to_reach:
                         
                    # Rockets!! going to goal
                    self.tb4_mode = "goal_mode"
 

                return             
         
        # following the wall depeding on the wall state
        distance = self.tb4_wall_distance
         
        if self.leftfront > distance and self.front > distance and self.rightfront > distance:
            self.tb4_wall_state = "identify_wall"
            msg.linear.x = self.linear_velocity
            msg.angular.z = -self.smooth_turn # turn right
             
        elif self.leftfront > distance and self.front < distance and self.rightfront > distance:
            self.tb4_wall_state = "turn"
            msg.angular.z = self.sharp_turn
             
             
        elif (self.leftfront > distance and self.front > distance and self.rightfront < distance):
            if (self.rightfront < self.wall_collision_avoidance):
                # reach wall
                self.tb4_wall_state = "turn"
                msg.linear.x = self.linear_velocity
                msg.angular.z = self.sharp_turn      
            else:           
                # straight_to_goal ahead
                self.tb4_wall_state = "along_wall" 
                msg.linear.x = self.linear_velocity   
                                     
        elif self.leftfront < distance and self.front > distance and self.rightfront > distance:
            self.tb4_wall_state = "identify_wall"
            msg.linear.x = self.linear_velocity
            msg.angular.z = -self.smooth_turn # turn right
             
        elif self.leftfront > distance and self.front < distance and self.rightfront < distance:
            self.tb4_wall_state = "turn"
            msg.angular.z = self.sharp_turn
             
        elif self.leftfront < distance and self.front < distance and self.rightfront > distance:
            self.tb4_wall_state = "turn"
            msg.angular.z = self.sharp_turn
             
        elif self.leftfront < distance and self.front < distance and self.rightfront < distance:
            self.tb4_wall_state = "turn"
            msg.angular.z = self.sharp_turn
             
        elif self.leftfront < distance and self.front > distance and self.rightfront < distance:
            self.tb4_wall_state = "identify_wall"
            msg.linear.x = self.linear_velocity
            msg.angular.z = -self.smooth_turn # turn right
             
        else:
            pass
 
        self.publisher_.publish(msg)    
         
    def bug2_algorithm(self):
     
        # Calculating sg line iteratively
        if self.is_SG_line_cal == False:
         
            # setting tb4 mode as goal mode
            self.tb4_mode = "goal_mode"            
 
            self.sg_line_xstart = self.x_pose_live
            self.sg_line_xgoal = self.goal_x[self.goal_id_lst]
            self.sg_line_ystart = self.y_pose_live
            self.sg_line_ygoal = self.goal_y[self.goal_id_lst]
             
            # estimating slope of sg line
            self.sg_line_slope = (
                (self.sg_line_ygoal - self.sg_line_ystart) / (
                self.sg_line_xgoal - self.sg_line_xstart))
             
            # estimating y intercept
            self.sg_line_y_intercept = self.sg_line_ygoal - (
                    self.sg_line_slope * self.sg_line_xgoal) 
                 
            self.is_SG_line_cal = True
             
        if self.tb4_mode == "goal_mode":
            self.path_planner()

        elif self.tb4_mode == "wall_mode":
            self.boundary_check()

def main(args=None):
 
    rclpy.init(args=args)
    turtlebot_4 = TurtleRide()
    rclpy.spin(turtlebot_4)
    turtlebot_4.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()