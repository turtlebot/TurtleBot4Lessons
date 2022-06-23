import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class TurtleRide(Node):

    def __init__(self):
        super().__init__('turtle_planner_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.turtle_planner)
        self.i = 0

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