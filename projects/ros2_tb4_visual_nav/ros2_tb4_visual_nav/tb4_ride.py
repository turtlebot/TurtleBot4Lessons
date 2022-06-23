from msilib.schema import SelfReg
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from .tb4_bug2_planner import tb4_pathplanner

class TurtleRide(Node):

    def __init__(self):
        super().__init__('turtle_planner_node')
        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.2  # seconds
        # Call backs
        self.timer = self.create_timer(timer_period, self.turtle_planner)
        self.i = 0
        # Creating objects for each phase of the Robot Navugation
        self.tb4_pathplanner = tb4_pathplanner()
        # Subscribers
        sub_laser = self.create_subscription('/scan', LaserScan, self.laser_callback)

    def laser_callback(self):
        global zones
        zones = {
            'right':  min(min(self.msg.ranges[0:143]), 10),
            'fright': min(min(self.msg.ranges[144:287]), 10),
            'front':  min(min(self.msg.ranges[288:431]), 10),
            'fleft':  min(min(self.msg.ranges[432:575]), 10),
            'left':   min(min(self.msg.ranges[576:719]), 10),
        }

    def turtle_planner(self):
        msg = String()
        msg.data = 'Hello Turtlebot 4: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        
def main(args=None):

    rclpy.init(args=args)
    turtle_ride = TurtleRide()
    rclpy.spin(turtle_ride)
    turtle_ride.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()