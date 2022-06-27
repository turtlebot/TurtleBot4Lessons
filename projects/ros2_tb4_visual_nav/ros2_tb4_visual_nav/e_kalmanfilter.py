import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray 
import math
import numpy as np
 
class KalmanFilter(Node):

    def __init__(self):
        super().__init__('KalmanFilter')

        # Subscribers
        self.vel_sub = self.create_subscription(Twist,'/cmd_vel',self.command_velocity, 10)
        self.odom_sub = self.create_subscription(Odometry,'/odom',self.odometry_data,10)
         
        # Publishers
        self.ekf_pub = self.create_publisher(Float64MultiArray,'/state_estimator',10)
             
        
    def command_velocity(self, msg):
        """
        callback method to listen velocity commands
        Input velocity vector [linear_vel,linear_vel,angular_vel] 
        """
        # Forward velocity in the x direction robot's reference frame
        linear_vel = msg.linear.x
         
        # Angular velocity around the robot's z axis
        angular_vel = msg.angular.z
         
        # [linear_vel,linear_vel,angular_vel]        
        self.input_velocity[0] = linear_vel
        self.input_velocity[1] = linear_vel
        self.input_velocity[2] = angular_vel
 
    def odometry_data(self, msg):
        """
        subscribing to odometry data- the robot's position and orientation in the global reference frame
        The position is x, y, z.
        The orientation is a x,y,z,w quaternion. 
        """                    
        roll, pitch, yaw = self.euler_from_quaternion(
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w)
                 
        state_vector = [msg.pose.pose.position.x,msg.pose.pose.position.y,yaw]

        observation_vector =  np.array([state_vector[0],
                        state_vector[1],
                        state_vector[2]])
         
        # Applying the Extended Kalman Filter        
        updated_state_estimate = self.ekf(observation_vector)
         
        # Publish the estimate state
        self.publish_estimated_state(updated_state_estimate)
 
    def ekf(self, obs_vector):
        #[TODO]
        return 
 
def main(args=None):
    """
    Entry point for the progam.
    """
     
    # Initialize rclpy library
    rclpy.init(args=args)
 
    # Create the node
    estimator = KalmanFilter()
 
    # Spin the node so the callback function is called.
    # Pull messages from any topics this node is subscribed to.
    # Publish any pending messages to the topics.
    rclpy.spin(estimator)
 
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    estimator.destroy_node()
     
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()