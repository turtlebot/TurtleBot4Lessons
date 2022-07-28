import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 # OpenCV library
import mediapipe as mp
import numpy as np
from matplotlib import pyplot as plt
import time

class TurtlePerception3D(Node):
    """
    - TurtlePerception3D class inherits from (or is a subclass of) Node attributes as a ROS Node
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
        TurtlePerception3D class constructor to initialize nodes,
        subscribers, publishers and parameters
        """
        super().__init__('tb4_perception_3d')

        # subscriber to receive an Image from the /color/image topic.
        self.subscription = self.create_subscription(Image,'/color/image',
                                                      self.perception_callback, 10)

        # Initialize bridge between ROS2 and OpenCV
        self.bridge = CvBridge()
        self.mp_objectron = mp.solutions.objectron
        self.mp_drawing = mp.solutions.drawing_utils

    def perception_callback(self, frames_data):
        """
        TurtlePerception3D class constructor to initialize nodes,
        subscribers, publishers and parameters

        Args: frames_data
        """
        try:
            self.get_logger().info('Receiving Turtlebot4 visual frames_data')

            # current frame
            current_frame = self.bridge.imgmsg_to_cv2(frames_data, desired_encoding="bgr8")

            resized_image = cv2.resize(current_frame, (720, 480))

            # Poping each and every frame
            cv2.imshow("TurtleBot4 Camera View", resized_image)

            process_frames = resized_image.copy()

            with self.mp_objectron.Objectron(static_image_mode=False,
                                            max_num_objects=2,
                                            min_detection_confidence=0.5,
                                            min_tracking_confidence=0.8,
                                            model_name='Shoe') as objectron:
                
                start = time.time()

                #Convert the BGR image to RGB
                image = cv2.cvtColor(process_frames, cv2.COLOR_BGR2RGB)

                # Improving performace by marking the image as not writeable to pass by reference
                image.flags.writeable = False
                
                # 3D object detection
                detected_object_frame = objectron.process(image)

                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                if detected_object_frame.detected_objects:
                    for  detected_objects in detected_object_frame.detected_objects:
                        self.mp_drawing.draw_landmarks(image, detected_objects.landmarks_2d, self.mp_objectron.BOX_CONNECTIONS)
                        self.mp_drawing.draw_axis(image, detected_objects.rotation, detected_objects.translation)

                end = time.time()
                total_time = end - start

                fps = 1 / total_time
                cv2.putText(image, f'FPS: {int(fps)}', (20,70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 2)
                # displaying the image after drawing contours
                cv2.imshow('Turtlebot 4 Simple Object Detection', image)
                cv2.waitKey(1)
        
        except CvBridgeError as e:
            print(e)

def main(args=None):
    """
    Main method to instantiate ROS nodes and TurtlePerception3D class to publish image
    processed object detection

    Args:
        None
    """
    rclpy.init(args=args)
    turtle_perception_3d = TurtlePerception3D()
    rclpy.spin(turtle_perception_3d)
    turtle_perception_3d.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
