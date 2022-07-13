import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray 
import math
import numpy as np
 
class KalmanFilter(Node):
    """Summary of KalmanFilter class

    KalmanFilter class inherits from (or is a subclass of) Node
    Extended Kalman filter (EKF) is the nonlinear version of the Kalman filter
    which linearizes about an estimate of the current mean and covariance.

    Attributes:
        Node: Is a class from rclpy.node.Node(node_name, *, context=None,
        cli_args=None, namespace=None, use_global_arguments=True,
        enable_rosout=True, start_parameter_services=True, parameter_overrides=None,
        allow_undeclared_parameters=False, automatically_declare_parameters_from_overrides=False)
        used to create a node, publish/ subscribe a node and access other ROS2 features
        
        More Information at: https://docs.ros2.org/latest/api/rclpy/api/node.html
    """

    def __init__(self):
        """
        Initiate the Node class's constructor
        """
        super().__init__('KalmanFilter')

        # Subscribers
        self.vel_sub = self.create_subscription(Twist,'/cmd_vel',self.command_velocity, 10)
        self.odom_sub = self.create_subscription(Odometry,'/odom',self.odometry_data,10)
         
        # Publishers
        self.ekf_pub = self.create_publisher(Float64MultiArray,'/state_estimator',10)

        self.A_t_1 = np.array([[1.0,0,0],[0,1.0,0],[0,0,1.0]])
        self.state_vector_t_1 = np.array([0.0,0.0,0.0])
        self.control_input_velocity = np.array([0.001,0.001,0.001])
        self.process_noise_v_t_1 = np.array([0.088,0.088,0.026])
        self.H_t = np.array([[1.0,0,0],[0,1.0,0],[0,0,1.0]])
        self.sensor_noise_w_t = np.array([0.06,0.06,0.03]) 
        self.d_t = 0.002
        self.est_yaw_angle = 0.0
        self.P_t_1 = np.array([[0.1,0,0],[0,0.1,0],[0,0,0.1]])
        self.Q_t = np.array([[1.0,0,0],[0,1.0,0],[0,0,1.0]])
        self.R_t = np.array([[1.0,0,0],[0, 1.0,0],[0,0,1.0]])

    def command_velocity(self, msg):
        """
        A callback method to listen velocity commands of the turtlebot4
        Args:
            msg:
                Type: geometry_msgs/Twist.msg
                Description: This expresses velocity in free space broken into its linear and angular parts.
                    Vector3  linear
                    Vector3  angular
        
        Inputs: Forward velocity in the x direction robot's reference frame and 
                Angular velocity around the robot's z axis from geometry_msgs/Twist.msg

        Output: velocity vector [linear_vel, linear_vel, angular_vel] 

        """
        # Forward velocity in the x direction robot's reference frame
        linear_vel = msg.linear.x
         
        # Angular velocity around the robot's z axis
        angular_vel = msg.angular.z
         
        # [linear_vel, linear_vel, angular_vel]        
        self.control_input_velocity[0] = linear_vel
        self.control_input_velocity[1] = linear_vel
        self.control_input_velocity[2] = angular_vel
 
    def odometry_data(self, msg):
        """
        - A callback method for subscribing to odometry data - the robot's position and orientation in the global reference frame
        - Convert odom data into observation state vector
        - estimate states from extended kalman filter [x, y, yaw]

        Args:
            msg:
                Type: nav_msgs/msg/Odometry.msg
                Description: This represents an estimate of a position and velocity in free space.
                1. The pose in this message should be specified in the coordinate frame given by header.frame_id
                2. The twist in this message should be specified in the coordinate frame given by the child_frame_id
        
        Inputs: The position is x, y, z.
                The orientation is a x,y,z,w quaternion.

        Output: State estimates from estimated kalman filter ---> [x, y, yaw] 

        """
        # Converting quaternion values to euler angle (yaw)                   
        roll, pitch, yaw = self.quaternion_to_euler(
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w)
        
        # Input observation state vector [x, y, yaw]
        state_vector = [msg.pose.pose.position.x,msg.pose.pose.position.y,yaw]

        # Converting state vector into numpy array
        observation_vector =  np.array([state_vector[0], state_vector[1], state_vector[2]])
         
        # Applying the Extended Kalman Filter        
        updated_state_estimate = self.ekf(observation_vector)
         
        # Publish the estimate state
        self.pub_estimated_state(updated_state_estimate)
    
    def quaternion_to_euler(self, x, y, z, w):
        """
        - A method to convert robots odometry data from quaternion to euler angles

        Args:
            x: orientation along x-direction
            y: orientation along y-direction
            z: orientation along z-direction
            w: the scalar (real) part
        
        Inputs: The orientation is a x,y,z,w quaternion.

        Output: Euler angles
                roll_x: Rotation about the x axis = roll angle
                pitch_y: Rotation about the y-axis = pitch angle
                yaw_z: Rotation about the z-axis = yaw angle

        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z
    
    def bMatrix(self,yaw,dt):
        """
        To calculate B-Matrix
        Input:
             yaw: yaw angle -> Rotation about the z-axis
             dt = differential time
            
        Return:
            B: [3*3] B Matrix

        """
        B = np.array([ [np.cos(yaw) * dt,0,0],[0,np.sin(yaw)* dt,0], [0,0,dt]])                 
        return B

    def pub_estimated_state(self, state_vector):
        
        msg = Float64MultiArray()
        msg.data = state_vector
        self.ekf_pub.publish(msg)

    def ekf(self, obs_vector):
        """
        Extended Kalman Filter to return optimal states by calculating
        current mean and covariance

        Args:
            obs_vector: observation state vector from odometry_data callback method [x, y, yaw]
        
        Inputs: 
            obs_vector: Input observation state vector [x, y, yaw] 3x1 from odometry 

        Output: 
            state_estimate_t: State estimates at time interval t ---> [x, y, yaw] 3x1   
        """
        #Based on the state estimate at time t-1 and the control input applied
        # at time t-1, predict the state estimate at time t

        state_estimate_t = self.A_t_1 @ (
            self.state_vector_t_1) + (
            self.bMatrix(self.est_yaw_angle,self.d_t)) @ (
            self.control_input_velocity) + (
            self.process_noise_v_t_1)
             
        #Estimate the state covariance using the prior covariance and noise.
        P_t = self.A_t_1 @ self.P_t_1 @ self.A_t_1.T + (
            self.Q_t)
         
        # Error correction
        measurement_residual_y_t = obs_vector - (
            (self.H_t @ state_estimate_t) + (
            self.sensor_noise_w_t))
             
        # measurement residual covariance
        S_t = self.H_t @ P_t @ self.H_t.T + self.R_t
         
        # near-optimal Kalman gain
        K_t = P_t @ self.H_t.T @ np.linalg.pinv(S_t)
         
        # updated state estimate for time t
        state_estimate_t = state_estimate_t + (K_t @ measurement_residual_y_t)
         
        # Update the state covariance estimate for time t
        P_t = P_t - (K_t @ self.H_t @ P_t)
              
        # Update the estimated yaw      
        self.est_yaw_angle = state_estimate_t[2]
         
        # Update the state vector for t-1
        self.state_vector_t_1 = state_estimate_t
         
        # Update the state covariance matrix
        self.P_t_1 = P_t
         
        state_estimate_t = state_estimate_t.tolist()
        return state_estimate_t 
 
def main(args=None):

    rclpy.init(args=args)
    ekf_estimator = KalmanFilter()
    rclpy.spin(ekf_estimator)
    ekf_estimator.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()