import numpy as np 
import math 
import argparse
import rclpy 
from matplotlib.pyplot import get
from time import sleep 
from rclpy.node import Node
from std_msgs.msg import String 
from geometry_msgs.msg import Twist            
from sensor_msgs.msg import LaserScan    
from geometry_msgs.msg import Pose 
from std_msgs.msg import Float64MultiArray
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from irobot_create_msgs.msg import Dock
from irobot_create_msgs.action import DockServo, Undock, DriveDistance, RotateAngle

class TurtleUnDock(Node):
    """
    The class TurtleUnDock, attributes as a ROS Node that acts as a primary entrypoint
        in the ROS system for communication especially for undocking TB4.

    Attributes:
        Node: Is a class from rclpy.node.Node
            More Information at: https://docs.ros2.org/latest/api/rclpy/api/node.html

    Topics:
        - Publishers:
                - Topic name: /dock
                    - Topic type: irobot_create_msgs/msg/Dock
                    - Topic desciption: To publish dock status
                        Consists of boolean flags of dock status and visibity
                            header:
                                stamp:
                                    sec: 0
                                    nanosec: 0
                                frame_id: ''
                                dock_visible: true
                                is_docked: true
        - Subscribers:
                - Topic name: /dock
                    - Topic type: irobot_create_msgs/msg/Dock
                    - Topic desciption: To subcribe to dock status
                        Consists of boolean flags of dock status and visibity
                            header:
                                stamp:
                                    sec: 0
                                    nanosec: 0
                                frame_id: ''
                                dock_visible: true
                                is_docked: true
        - Actions:
                - ActionClient name: /undock
                - ActionClient desciption:
                                Perform Undock implementation

    """
    def __init__(self):
        """
        TurtleUnDock constructor to initialize nodes, subscribers, publishers and parameters
        """
        super().__init__('turtlebot4_undock')

        # Publishing Dock Status
        self.publisher_dock = self.create_publisher(Dock,'/dock',10)
        # Subsribing to Dock Status
        self.subscription_dock = self.create_subscription(Dock,'/dock',self.dock_status,qos_profile_sensor_data)
        # Undock Action client
        self.undock_action_client = ActionClient(self, Undock, '/undock')
        # Flag to know the Turtlebot4 dock status
        self.is_docked = True
        self.dock_flag = True

    def dock_action(self):
        """
        - Method to perform undock action using undock_action_client
        - The ActionClient.send_goal_async() method returns a future to a goal handle

        Args: None

        Output: dock_result = False
        """
        self.get_logger().info('Turtlebot4 dock status: Docked')
        undock_goal_msg = Undock.Goal()
        self.get_logger().info('::::::::::Undocking:::::::::::::')
        self.undock_action_client.wait_for_server()
        # Send goal to server and returns a future to a goal handle
        self._send_goal_result = self.undock_action_client.send_goal_async(undock_goal_msg, feedback_callback=self.feedback_callback)
        # Here we register a callback for when the future is complete
        self._send_goal_result.add_done_callback(self.goal_response_callback)

    def dock_status(self, msg):
        """
        - A method to get dock status of the turtlebot4

        Args:
            msg:
                Type: irobot_create_msgs/msg/Dock
                Description: Consists of boolean flags of dock status and visibity
                    header:
                        stamp:
                            sec: 0
                            nanosec: 0
                        frame_id: ''
                        dock_visible: true
                        is_docked: true

        Output: is_docked (True/False)
        """
        self.is_docked = msg.is_docked

    def goal_response_callback(self, future):
        """
        A callback function to check to see if the goal to server was rejected and return early

        Args:
            future
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        A callback function to log the result sequence

        Args:
            future
        """
        result = future.result().result
        self.get_logger().info('Docking Status: Dock {0}'.format(result.is_docked))
        self.get_logger().info('Turtlebot4 is ready for the Motion Planning')

    def feedback_callback(self, feedback_msg):
        """
        - A callback function for feedback messages received from
            send_goal_async() of Action Server-Client
        - In the callback we get the feedback portion of the message and
             print the partial_sequence field to the screen

        Args:
            feedback_msg
        """
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

def main(args=None):
    """
    Main method to instantiate ROS nodes and TurtleUnDock class to undock the bot

    Args:
        None
    """
    rclpy.init(args=args)
    turtlebot_4_undock = TurtleUnDock()
    turtlebot_4_undock.dock_action()
    rclpy.spin(turtlebot_4_undock)
    turtlebot_4_undock.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
