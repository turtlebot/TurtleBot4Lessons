import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2 # OpenCV library
import numpy as np
from matplotlib import pyplot as plt

class TurtlePerception(Node):
    """
    TurtlePerception class inherits from (or is a subclass of) Node

    Attributes:
        Node: Is a class from rclpy.node.Node(node_name, *, context=None,
        cli_args=None, namespace=None, use_global_arguments=True,
        enable_rosout=True, start_parameter_services=True, parameter_overrides=None,
        allow_undeclared_parameters=False, automatically_declare_parameters_from_overrides=False)
        used to create a node, publish/ subscribe a node and access other ROS2 features

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

    def perception_callback(self, frames_data):
        """
        TurtlePerception class constructor to initialize nodes,
        subscribers, publishers and parameters

        Args: frames_data
        """
        self.get_logger().info('Receiving Turtlebot4 visual frames_data')

        # current frame
        current_frame = self.bridge.imgmsg_to_cv2(frames_data)

        # Poping each and every frame
        cv2.imshow("TurtleBot4 Camera View", current_frame)

        process_frames = current_frame.copy()

        # converting image into grayscale image
        gray = cv2.cvtColor(process_frames, cv2.COLOR_BGR2GRAY)

        # setting threshold of gray image
        _, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

        # using a findContours() function

        # The function retrieves contours from the binary image using the algorithm
        # # (Satoshi Suzuki and others.Topological structural analysis of digitized binary images
        # by border following Computer Vision, Graphics, and Image Processing, 30(1):32–46, 1985.)
        # The contours are a useful tool for shape analysis and object detection and recognition.

        contours, _ = cv2.findContours(
            threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        i = 0

        # list for storing names of shapes
        for contour in contours:

            # here we are ignoring first counter because
            # findcontour function detects whole image as shape
            if i == 0:
                i = 1
                continue

            # cv2.approxPloyDP() function to approximate the shape
            approx = cv2.approxPolyDP(
                contour, 0.01 * cv2.arcLength(contour, True), True)

            # using drawContours() function
            cv2.drawContours(process_frames, [contour], 0, (0, 0, 255), 5)

            # finding center point of shape
            cv_moments = cv2.moments(contour)
            if cv_moments['m00'] != 0.0:
                x_point = int(cv_moments['m10']/cv_moments['m00'])
                y_point = int(cv_moments['m01']/cv_moments['m00'])

            # putting shape name at center of each shape
            if len(approx) == 3:
                cv2.putText(process_frames, 'Triangle', (x_point, y_point),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            elif len(approx) == 4:
                cv2.putText(process_frames, 'Quadrilateral', (x_point, y_point),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            elif len(approx) == 5:
                cv2.putText(process_frames, 'Pentagon', (x_point, y_point),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            elif len(approx) == 6:
                cv2.putText(process_frames, 'Hexagon', (x_point, y_point),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            else:
                cv2.putText(process_frames, 'circle', (x_point, y_point),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

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
