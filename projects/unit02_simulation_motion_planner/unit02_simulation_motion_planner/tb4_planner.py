import math
import time

from flask import flash
import rclpy
import numpy as np
from time import sleep 
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from rclpy.qos import qos_profile_sensor_data

#[TODO] Undocking before planning

class TurtlePlanner(Node):
    """
    - The class TurtlePlanner, attributes as a ROS Node that acts as a primary entrypoint
        in the ROS system for communication. It can be used to create ROS entities
        such as publishers, subscribers, services, etc.
    - This class is an implementation of the Motion Planner Algorithm.
    - Motion Planner Algorithm in a nutshell:
    (Most of the text books refer this algorithm as Bug2 planning algorithm)
            - Head towards goal on the sg-line (A line between start and goal points)
            - If an obstacle/wall is on the way, follow it until you encounter the sg-line
                 closer to the goal.
            - consider the point where turtlebot4 reaches the sg-line first as
                reach point. Similarly, consider the point where turtlebot4 leaves the
                sg-lines as leave point
            - Leave the obstacle/wall and continue toward the goal

    Attributes:
        Node: Is a class from rclpy.node.Node(node_name, *, context=None,
        cli_args=None, namespace=None, use_global_arguments=True,
        enable_rosout=True, start_parameter_services=True, parameter_overrides=None,
        allow_undeclared_parameters=False, automatically_declare_parameters_from_overrides=False)
        used to create a node, publish/ subscribe a node and access other ROS2 features

            More Information at: https://docs.ros2.org/latest/api/rclpy/api/node.html

    Topics:
    - Publishers:
            - Topic name: /cmd_vel
                - Topic type: geometry_msgs.msg/Twist
                - Topic desciption: To publish position and orientation of the turtlebot4
    - Subcribers:
            - Topic name: /scan
                - Topic type: sensor_msgs.msg/LaserScan
                - Topic desciption: subscribe to lidar data per cycle of the turtlebot4
            - Topic name: /state_estimator
                - Topic type: std_msgs.msg/Float64MultiArray
                - Topic desciption: subscribe to ekf state estimator of the turtlebot4
            - Topic name: /goal
                - Topic type: geometry_msgs.msg/Pose
                - Topic desciption: subscribe to goal pose [X Y]
    """
    def __init__(self):
        """
        TurtlePlanner constructor to initialize nodes, subscribers, publishers and parameters
        """
        super().__init__('turtlebot4_visual_nav')

        # Subscribing to Lidar data
        self.scan_subscriber = self.create_subscription(LaserScan,'/scan',self.lidar_scan,
                                                        qos_profile=qos_profile_sensor_data)

        # Subscribing to State estimator
        self.subscription = self.create_subscription(Float64MultiArray,'/state_estimator',
                                                        self.state_estimation,10)

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
        ### "turn": By default tb4 turns towards lidar["left"]
        ### "identify_wall": tb4 attempts to identify the wall
        ### "along_wall": tb4 moves algon to the wall with certain threshold

        self.tb4_wall_state = "turn"

        # Goal Position
        self.goal_x = False
        self.goal_y = False

        # Initializing each Lidar data cycle into zones
        # here we are breaking each lidar cycle in to 5 parts as
        # front, left, right, leftfront and rightfront to orient TB4 accordingly
        self.lidar = {}
        self.lidar["left"] = 9999999999.9
        self.lidar["leftfront"] = 9999999999.9
        self.lidar["front"] = 9999999999.9
        self.lidar["rightfront"] = 9999999999.9
        self.lidar["right"] = 9999999999.9

        # Essential Parameters

        # Linear velocity of TB4
        self.linear_velocity = 0.75

        # Angular velocity of TB4
        self.angular_velocity = 1.0

        # Angular turn velocity of TB4
        self.angular_turn_velocity = 0.5

        # live position and orientation of the robot in the world frame
        self.x_pose_live = 0.0
        self.y_pose_live = 0.0
        self.orientation_live = 0.0

        # Proximity between TB4 and an obstacle in meters
        self.collision_avoidance_proximity = 1.5

        # List of all intermediate goal coordinates
        self.goal_id_lst = 0

        # angular precision +/- 3.0 degrees
        self.angle_threshold = 3.0 * (math.pi / 180)

        # TB4 to goal distance
        self.tb4_goal_dist = 0.1

        self.sharp_turn = 0.7
        self.smooth_turn = 0.5

        # minimum wall distance tb4 can walkthrough
        self.tb4_wall_distance = 1.5

        # Collision avoidance theshold with wall
        self.wall_collision_avoidance = 1

        self.is_bug2 = True

        # Flag to calculate slope line between start and goal
        self.is_sg_line_cal = False

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
        self.dist_leave_to_reach = 0.2

        # minimum distance between TB4 and sg line
        self.min_distance_to_sg_line = 0.06




    def goal_estimation(self,msg):
        """
        A callback method for subscribing to goal pose - the robot's final
        goal position and orientation in the global reference frame

        Args:
            msg:
                Type: geometry_msgs/Pose.msg
                Description: A representation of pose in free space,
                composed of position and orientation.

        Output: goal_x, goal_y -> goal coordinates for turtlebot4
        """

        self.goal_x = [msg.position.x]
        self.goal_y = [msg.position.y]

    def lidar_scan(self, msg):
        """
        - lidar_scan is a callback method to capture lidar data per each cycle.
        - This method is also responsible to perform obstacle avoidance with Lidar and visual data

        Args:
            msg:
                Type: sensor_msgs/LaserScan.msg
                Description: Single scan from a planar laser range-finder.
                 If you have another ranging device with different behavior (e.g. a sonar array),
                 please find or create a different message, since applications
                 will make fairly laser-specific assumptions about this data

        Output: goal_x, goal_y -> goal coordinates for turtlebot4
        """
        self.lidar["left"] = msg.ranges[180]
        self.lidar["leftfront"] = msg.ranges[135]
        self.lidar["front"] = msg.ranges[90]
        self.lidar["rightfront"] = msg.ranges[45]
        self.lidar["right"] = msg.ranges[0]
        #[TODO] perecption based avoidance
        # if self.tb4_mode == "avoid_mode":
        #     self.obstacle_avoidance()

    def state_estimation(self, msg):
        """
        - Method to estimate the current position and orientation of the turtlebot4
        - Run motion planning algorithm upon receiving goal pose

        Args:
            msg:
                Type: std_msgs/Float64MultiArray.msg
                Description: float64[] data which consists of current pose and orientation
                            of the TB4

        Output: current x, y position and orientation of turtlebot4
        """
        # Storing current pose and orientation as current state
        current_state = msg.data
        self.x_pose_live = current_state[0]
        self.y_pose_live = current_state[1]
        self.orientation_live = current_state[2]

        # Condition to receive goal coordinates from /goal publisher
        if self.goal_x is False and self.goal_y is False:
            return

        # Run bug2_algorithm Motion Planner

        if self.is_bug2 is True:
            self.bug2_algorithm()
        else:
            self.get_logger().info('No Motion Planner is assigned')

    def obstacle_avoidance(self):
        """
        [TODO] Method to avoid static obstacles from lidar data + stereo camera
        """
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        # publish to /cmd_vel
        self.publisher_.publish(msg)
  
    def desired_yaw(self):
        """
         Method to estimate desired yaw along the sg-line towards goal pose
        """
        desired_yaw_angle = math.atan2(
                self.goal_y[self.goal_id_lst] - self.y_pose_live,
                self.goal_x[self.goal_id_lst] - self.x_pose_live)
        return desired_yaw_angle

    def adjust_orientation(self):
        """
        - Method to estimate the orientation of the turtlebot4
        - Adjust the orientation towards goal along the sg-line

        Args: None

        Output:
            msg.angular.z -> publish desired yaw angle
        """
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.get_logger().info('Entering orient to goal mode')

        # calculate the estimate yaw angle depending on the current pose and goal pose
        yaw_error = self.desired_yaw() - self.orientation_live

        # change the orientation if error is greater than anticipated threshold
        if math.fabs(yaw_error) > self.angle_threshold:
            self.get_logger().info('changing orientation')
            if yaw_error > 0:
                # turn lidar["left"]
                msg.angular.z = self.angular_turn_velocity
            else:
                # Turn lidar["right"]
                msg.angular.z = -self.angular_turn_velocity

            # Publish command velocity acoordingly
            self.publisher_.publish(msg)

        # If the orentation is good change the goal state to straight
        else:
            self.tb4_goal_state = "straight_to_goal"

            # Publish command velocity acoordingly
            self.publisher_.publish(msg)

    def to_goal(self):
        """
        - Method to estimate the correct pose of the turtlebot4 to reach goal
        - Adjust the pose by applying linear velocity towards goal and simultaneously
            recalcuate desired yaw to adjust orientation

        Args: None

        Output:
            - msg.linear.x -> publish desired Linear velocity along x direction
            - msg.angular.z -> publish desired yaw angle
        """
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.get_logger().info('Entering straight to goal mode')
        pos_error = math.sqrt(pow(self.goal_x[self.goal_id_lst] - self.x_pose_live, 2)+ pow(self.goal_y[self.goal_id_lst] - self.y_pose_live, 2))
        # estimate distance between goal and current location then repeat the process
        if pos_error > self.tb4_goal_dist:
            self.get_logger().info('estimating distance between goal and current location')
            # Apply linear velocity
            msg.linear.x = self.linear_velocity

            # Publish command velocity acoordingly to move the robot
            self.publisher_.publish(msg)

            # check if orientation is good
            yaw_error = self.desired_yaw() - self.orientation_live
    
            # estimating if orientation is reasonable else changing
            # the state of the turtlebot4
            if math.fabs(yaw_error) > self.angle_threshold:
                self.get_logger().info('switching goal state to orient_to_goal')
                # Changing the state of the tb4 to adjust orentation
                self.tb4_goal_state = "orient_to_goal"

        # else change the goal state to goal state reached
        else:
            # Change the state
            self.tb4_goal_state = "goal_state"

            # Publish command velocity with 0 to stop
            self.publisher_.publish(msg)
  
    def reached_goal(self):
        """
        - Method to predict the goal state and its status

        Args: None

        Output:
            - Goal status
        """
        self.get_logger().info('Entering goal state mode')
        self.get_logger().info('goal_state! X:%f Y:%f' % (
            self.goal_x[self.goal_id_lst],
            self.goal_y[self.goal_id_lst]))
        # incrementing to next goal state
        self.goal_id_lst = self.goal_id_lst + 1

        # reaching final goal

        if (self.goal_x[self.goal_id_lst] is self.goal_x and self.goal_y[self.goal_id_lst] is self.goal_y):
            self.get_logger().info('Goal Reached!!! Planning complete')


        # If not
        else:
            # change the goal state to ajdust orientation
            self.tb4_goal_state = "orient_to_goal"
        self.is_sg_line_cal = False

    def path_planner(self):
        """
        - Method to plan the path of turtlebot4 towards goal position. The plan is calculated
        depending on the following conditions
            - Initially, the robot drives by adjusting its goal states to reach the obstacle/ wall
                along the sg-line towards the goal. Following are the tb4 goal
                states during this phase:
                - "orient_to_goal": adjusts its orientation towards the goal pose along the sg-line
                     by calculating yaw error
                - "straight_to_goal": Apply linear and angular velocity to reach goal by
                recalculating yaw and its position at each time interval and comparing it
                with goal pose
                - "goal_state": compare current pose with goal pose and
                    confrim that goal is reached.
            - Once the wall is reached:
                - Cheking for wall:
                    - Depending on the lidar data and defined wall avoidance proximity. The mode
                        of the TB4 is changed to wall_mode
                - During the wall_mode TB4 activates boundary_check method and follows along
                  the wall/ obstacle

        Args: None

        Output: Change modes of TB4 according to the goal states
        """
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        if self.is_bug2 is True:

            # Cheking for wall
            distance = 0.5
            self.get_logger().info('Planning the path to goal pose')
            if (self.lidar["leftfront"] < distance or self.lidar["front"] < distance or self.lidar["rightfront"] < distance):
                self.get_logger().info('Switched to wall mode')
                # If there is wall then change the tb4 mode
                self.tb4_mode = "wall_mode"

                # Point at which Turtle bot 4 reaches the wall and follows the wall 
                self.reach_x = self.x_pose_live
                self.reach_y = self.y_pose_live

                # Find the distance from reach point to goal point
                self.dist_reach_to_goal = (math.sqrt((pow(self.goal_x[self.goal_id_lst] - self.reach_x, 2)) +
                (pow(self.goal_y[self.goal_id_lst] - self.reach_y, 2))))

                # Perform a stop and sharp turn to follow the wall
                self.get_logger().info('Taking a quick turn')
                msg.angular.z = self.sharp_turn
                # publish new msg to command velocity
                self.publisher_.publish(msg)
                self.get_logger().info('Exiting wall check condition')
                # Exit this function
                return

            # Turn the orientation of the tb4 accordingly to reach the goal pose
            if (self.tb4_goal_state == "orient_to_goal"):
                self.adjust_orientation()

            # straight_to_goal
            elif (self.tb4_goal_state == "straight_to_goal"):
                self.to_goal()

            # goal_state
            elif (self.tb4_goal_state == "goal_state"):
                self.reached_goal()

            else:
                self.get_logger().info('Reached Unknown goal state')


    def boundary_check(self):
        """
        - This method enables the turtlebot4 to follow the boundary of a wall
            depending on the wall state:
            - "identify_wall": tb4 attempts to identify the wall from the data
                 received from Lidar and other conditions
            - "along_wall": tb4 moves along the wall with certain collison avoidance threshold
            - "turn": By default tb4 turns towards left of the wall


        Args: None

        Output: Change modes of TB4 according to the wall states
        """

        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        if self.is_bug2 is True:
            self.get_logger().info('Checking boundary conditions')
            # Calculating the closest point on sg line
            x_start_goal_line = self.x_pose_live
            y_start_goal_line = (self.sg_line_slope * (x_start_goal_line)) + (self.sg_line_y_intercept)
            # calculating distance between claculated point
            distance_to_start_goal_line = 0.05 + math.sqrt(pow(x_start_goal_line - self.x_pose_live, 2) + pow(y_start_goal_line - self.y_pose_live, 2)) 

            self.get_logger().info('Distance to SG Line: %f' % distance_to_start_goal_line)
            # what if hit the sg line again? on the second collison with sg-line
            # we consider the tb4 to leaving the line
            if distance_to_start_goal_line < self.min_distance_to_sg_line:
                self.get_logger().info('evaluating condition to switch mode')
                # storing the current pose of tbr at that point
                self.leave_x = self.x_pose_live
                self.leave_y = self.y_pose_live
                # estimating the distance between goal and leave point
                self.dist_leave_to_goal = math.sqrt(pow(self.goal_x[self.goal_id_lst] -
                 self.leave_x, 2)+ pow(self.goal_y[self.goal_id_lst] - self.leave_y, 2))
                # check for diffrentiating leave point and reach point.
                # if the distance to goal is less then it is reach point
                difference = self.dist_reach_to_goal - self.dist_leave_to_goal
                self.get_logger().info('Difference: %f' % difference)
                if difference > self.dist_leave_to_reach:
                    # Rockets!! going to goal
                    self.tb4_mode = "goal_mode"
                return

            # following the wall depending on the wall state
            self.get_logger().info('Avoiding collision with wall and following wall')
            if( self.lidar["leftfront"] > self.tb4_wall_distance and
                    self.lidar["front"] > self.tb4_wall_distance and
                    self.lidar["rightfront"] > self.tb4_wall_distance):

                self.tb4_wall_state = "identify_wall"
                self.get_logger().info('Applying linear velocity and turn right')
                msg.linear.x = self.linear_velocity
                msg.angular.z = -self.smooth_turn # turn right

            elif (self.lidar["leftfront"] > self.tb4_wall_distance and
                    self.lidar["front"] < self.tb4_wall_distance and
                    self.lidar["rightfront"] > self.tb4_wall_distance):

                self.get_logger().info('Turning Sharp left')
                self.tb4_wall_state = "turn"
                msg.angular.z = self.sharp_turn # turn left

            elif (self.lidar["leftfront"] > self.tb4_wall_distance and
                    self.lidar["front"] > self.tb4_wall_distance and
                    self.lidar["rightfront"] < self.tb4_wall_distance):

                if (self.lidar["rightfront"] < self.wall_collision_avoidance):
                    # reach wall
                    self.get_logger().info('Reached wall and Turning Sharp left')
                    self.tb4_wall_state = "turn"
                    msg.linear.x = self.linear_velocity
                    msg.angular.z = self.sharp_turn # turn left
                else:
                    # straight_to_goal ahead
                    self.tb4_wall_state = "along_wall"
                    msg.linear.x = self.linear_velocity # go straight

            elif (self.lidar["leftfront"] < self.tb4_wall_distance and
                    self.lidar["front"] > self.tb4_wall_distance and
                    self.lidar["rightfront"] > self.tb4_wall_distance):

                self.get_logger().info('Reach wall and turn right')
                self.tb4_wall_state = "identify_wall"
                msg.linear.x = self.linear_velocity
                msg.angular.z = -self.smooth_turn # turn right

            elif (self.lidar["leftfront"] > self.tb4_wall_distance and
                 self.lidar["front"] < self.tb4_wall_distance and 
                 self.lidar["rightfront"] < self.tb4_wall_distance):

                self.get_logger().info('turn left')
                self.tb4_wall_state = "turn"
                msg.angular.z = self.sharp_turn # turn left

            elif (self.lidar["leftfront"] < self.tb4_wall_distance and
                    self.lidar["front"] < self.tb4_wall_distance and
                    self.lidar["rightfront"] > self.tb4_wall_distance):

                self.get_logger().info('turn left')
                self.tb4_wall_state = "turn"
                msg.angular.z = self.sharp_turn # turn left

            elif (self.lidar["leftfront"] < self.tb4_wall_distance and
                    self.lidar["front"] < self.tb4_wall_distance and
                    self.lidar["rightfront"] < self.tb4_wall_distance):

                self.get_logger().info('turn left')
                self.tb4_wall_state = "turn"
                msg.angular.z = self.sharp_turn #turn left

            elif (self.lidar["leftfront"] < self.tb4_wall_distance and
                    self.lidar["front"] > self.tb4_wall_distance and
                    self.lidar["rightfront"] < self.tb4_wall_distance):

                self.get_logger().info('turn right')
                self.tb4_wall_state = "identify_wall"
                msg.linear.x = self.linear_velocity
                msg.angular.z = -self.smooth_turn # turn right in clockwise

            else:
                self.get_logger().info('Unknown condition reached')


        self.publisher_.publish(msg)

    def bug2_algorithm(self):
        """
        - Method to run bug2 motion planner algorithm
        - Algorithm in a nutshell:
            - Head towards goal on the sg-line (A line between start and goal points)
            - If an obstacle/wall is on the way, follow it until you encounter the sg-line
                 closer to the goal.
            - consider the point where turtlebot4 reaches the sg-line first as
                reach point. Similarly, consider the point where turtlebot4 leaves the
                sg-lines as leave point
            - Leave the obstacle/wall and continue toward the goal
        """
        # Calculating sg line iteratively
        if self.is_sg_line_cal is False:

            # setting tb4 mode as goal mode
            self.tb4_mode = "goal_mode"
            self.get_logger().info('Bug Algo active! Switching to goal mode')
            self.sg_line_xstart = self.x_pose_live
            self.sg_line_xgoal = self.goal_x[self.goal_id_lst]
            self.sg_line_ystart = self.y_pose_live
            self.sg_line_ygoal = self.goal_y[self.goal_id_lst]

            # estimating slope of sg line
            self.sg_line_slope = (
                (self.sg_line_ygoal - self.sg_line_ystart) / (
                self.sg_line_xgoal - self.sg_line_xstart))

            # estimating y intercept
            self.sg_line_y_intercept = self.sg_line_ygoal - (self.sg_line_slope * self.sg_line_xgoal)
            self.is_sg_line_cal = True

        if self.tb4_mode == "goal_mode":
            self.get_logger().info('Running Path Planner')
            self.path_planner()

        elif self.tb4_mode == "wall_mode":
            self.get_logger().info('Switched to wall mode. Running boundary check')
            self.boundary_check()

def main(args=None):
    """
    Main method to instantiate ROS nodes and TurtlePlanner class to trigger
    motion planner for Turtlebot4 upon given goal pose

    Args:
        None
    """
    rclpy.init(args=args)
    turtlebot_4 = TurtlePlanner()
    rclpy.spin(turtlebot_4)
    turtlebot_4.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()