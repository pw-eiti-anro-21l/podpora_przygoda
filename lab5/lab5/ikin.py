import sys
from interpolation.srv import Interpolation
import rclpy
from rclpy.node import Node


class jint(Node):

    def __init__(self):
        super().__init__('jint')
        self.cli = self.create_client(Interpolation, 'interpolacja')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Service is not available, waiting...')
        self.req = Interpolation.Request()
