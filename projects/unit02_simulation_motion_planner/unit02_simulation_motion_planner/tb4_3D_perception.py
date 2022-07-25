import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2 # OpenCV library
import mediapipe as mp
import numpy as np
from matplotlib import pyplot as plt

class TurtlePerception(Node):
    """
    - TurtlePerception class inherits from (or is a subclass of) Node attributes as a ROS Node
     that acts as a primary entrypoint in the ROS system for communication. It can be used to 
     create ROS entities such as publishers, subscribers, services, etc.
    - This class is an implementation of the 3D object detection using openCV and MediaPipe.
        Here, we use inbuilt opencv methods such as
         - TODO


    Attributes:
        Node: Is a class from rclpy.node.Node

            More Information at: https://docs.ros2.org/latest/api/rclpy/api/node.html 

    Topics:
        - Publishers: None
        - Subscribers:
                - Topic name: /color/image
                    - Topic type: sensor_msgs.msg/Image
                    - Topic desciption: To subcribe image frames from OKA-D camera sensor of the TB4
    """
    def __init__(self):
        """
        TurtlePerception class constructor to initialize nodes,
        subscribers, publishers and parameters
        """
        super().__init__('tb4_perception')

        # subscriber to receive an Image from the /color/image topic.
        self.subscription = self.create_subscription(Image,'/color/image',
                                                      self.perception_callback, 10)

        # Initialize bridge between ROS2 and OpenCV
        self.bridge = CvBridge()
        self.mp_objectron = mp.solutions.objectron
        self.mp_drawing = mp.solutions.drawing_utils

    def perception_callback(self, frames_data):
        """
        TurtlePerception class constructor to initialize nodes,
        subscribers, publishers and parameters

        Args: frames_data
        """
        self.get_logger().info('Receiving Turtlebot4 visual frames_data')

        # current frame
        current_frame = self.bridge.imgmsg_to_cv2(frames_data, desired_encoding="bgr8")

        resized_image = cv2.resize(current_frame, (360, 640))

        # Poping each and every frame
        cv2.imshow("TurtleBot4 Camera View", resized_image)

        process_frames = resized_image.copy()

        with self.mp_objectron.Objectron()

        # displaying the image after drawing contours
        cv2.imshow('Turtlebot 4 Simple Object Detection', process_frames)
        cv2.waitKey(1)

def main(args=None):
    """
    Main method to instantiate ROS nodes and TurtlePerception class to publish image
    processed object detection

    Args:
        None
    """
    rclpy.init(args=args)
    turtle_perception = TurtlePerception()
    rclpy.spin(turtle_perception)
    turtle_perception.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
