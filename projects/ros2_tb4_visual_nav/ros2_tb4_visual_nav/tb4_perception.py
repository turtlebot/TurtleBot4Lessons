import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2 # OpenCV library
 
class TurtlePerception(Node):
    """
    TurtlePerception class as a Node
    """
    def __init__(self):
 
        super().__init__('tb4_perception')
        
        # subscriber to receive an Image from the /color/image topic.
        self.subscription = self.create_subscription(Image,'video_frames', self.perception_callback, 10)
        self.subscription
        
        # Initialize bridge between ROS2 and OpenCV
        self.bridge = CvBridge()
   
    def perception_callback(self, frames_data):

    
        self.get_logger().info('Receiving Turtlebot4 visual frames_data')
    
        # current frame
        current_frame = self.bridge.imgmsg_to_cv2(frames_data)
        
        # Poping each and every frame
        cv2.imshow("TurtleBot4 Camera View", current_frame)
        
        cv2.waitKey(1)
  
def main(args=None):
  
  rclpy.init(args=args)
  turtle_perception = TurtlePerception()
  rclpy.spin(turtle_perception)
  turtle_perception.destroy_node()
  rclpy.shutdown()

  
if __name__ == '__main__':
  main()