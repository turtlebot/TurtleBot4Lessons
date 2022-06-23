# import ROS 2 libraries
import rclpy
from rclpy.node import Node
# import ROS 2 messages
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# import tf2
from tf2_ros import TransformBroadcaster
import tf_transformations
from math import pi,cos,sin
import numpy as np
from std_msgs.msg import String

class TurtleRide(Node):

    def __init__(self):

        super().init__("turtle_ride_node")

        # self.vel_pub = self.create_publisher(Twist,'/cmd_vel',10)
        # self.vel_msg = Twist()
        # timer_period = 0.2  # seconds
        # self.timer = self.create_timer(timer_period, self.turtle_rider)
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        # Subscribing to receive Pose and sensor data

         
    def turtle_rider(self):
        #[TODO] Path Panner
        #[TODO] Motional Planner
        #self.vel_pub.publish(self.vel_msg)
        # self.get_logger().info('Publishing: Test')
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        

def main(args=None):
    rclpy.init(args=args)
    turtle_ride = TurtleRide()
    rclpy.spin(turtle_ride)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
