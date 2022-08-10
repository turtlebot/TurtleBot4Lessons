from asyncio import FastChildWatcher
import string
from tkinter.messagebox import NO
import rclpy
from rclpy.node import Node
from turtlebot4_msgs.srv import MyProgram
import subprocess
from ament_index_python.packages import get_package_share_directory, get_package_prefix



class ProjectsManager(Node):
    """
    The class ProjectsManager, attributes as a ROS Node that acts as a primary entrypoint
        in the ROS system for communication especially for launching multiple packages from HMI display.

    Attributes:
        Node: Is a class from rclpy.node.Node
            More Information at: https://docs.ros2.org/latest/api/rclpy/api/node.html

    Topics:
        - Services:
                - Service name: /my_program
                - Service desciption:
                            - Create a service "my_program" to trigger a bool srv "my_program_on"
                            - When my_program_on is set to True from client node. The my_program service
                                returns Success

    """
    def __init__(self):
        """
        ProjectsManager constructor to initialize nodes, subscribers, publishers and parameters
        """
        super().__init__('my_program_service')
        self.my_program_request = self.create_service(MyProgram, 'my_program', self.project_manager_callback)
        self.response_message = ''
        self.response_result = False

    def project_manager_callback(self, request, response):
        
        self.response_message = response.message
        self.response_result = response.success
        # If the client requests the service with my_program_on = True, we run a bash script of launch
        # Then return Success message upon successful request exectution
        if(request.my_program_on):
            self.get_logger().info('My Program Mode is turned on')
            subprocess.call(["bash", '/home/sumedh/workspace/src/TurtleBot4Lessons/projects/hmi_simulation_project_manager/config/auto_launch.bash'])
            response.message = "My Program Mode received and running projects"
            response.success = True

        else:
            self.get_logger().info('My Program Mode rejected')
            response.message = "My Program Mode rejected and failed to run projects"
            response.success = False

        return response


def main(args=None):
    """
    Main method to instantiate ROS nodes and TurtleUnDock class to undock the bot

    Args:
        None
    """
    rclpy.init(args=args)

    projects_manager = ProjectsManager()

    rclpy.spin(projects_manager)

    rclpy.shutdown()


if __name__ == '__main__':
    main()