import rclpy
from rclpy.node import Node
import sys
import os
from time import sleep

from geometry_msgs.msg import Twist

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.declare_parameters(namespace='', parameters=[
            ('up', 'w'),
            ('down', 's'),
            ('left', 'a'),
            ('right', 'p')
        ])


    def timer_callback(self):
        msg = Twist()
        parameters = {parameter: self_get_parameter(parameter).get_parameter_value()
            .string_value for parameter in ['up', 'down', 'left', 'right']}
        key = stdscr.getkey()

        if(self.left == key):
            msg.angular.z = vel_ang
            self.get_logger().info('Turning left')
        elif(self.right == key):
            msg.angular.z = -vel_ang
            self.get_logger().info('Turning right')
        elif(self.up == key):
            msg.linear.x = vel_lin
            self.get_logger().info('Going forward')
        elif(self.down == key):
            msg.linear.x = -vel_lin
            self.get_logger().info('Going backward')
        else:
            print("Start moving by pressing one of keys: a d w s)

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collec.tor destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
