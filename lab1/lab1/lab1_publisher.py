import rclpy
from rclpy.node import Node
import sys
import os
import curses
from time import sleep

from geometry_msgs.msg import Twist

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.declare_parameters(namespace='', parameters=[
            ('up', 'w'),
            ('down', 's'),
            ('left', 'a'),
            ('right', 'd')
        ])

    def timer_callback(self):
        parameters = {parameter: self.get_parameter(parameter).get_parameter_value()
            .string_value for parameter in ['up', 'down', 'left', 'right']}
        sleep(0.2)
        stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        msg = Twist()
        vel_ang = 1.0
        vel_lin = 1.0
        key = stdscr.getkey()

        if elf.left == key:
            msg.angular.z = vel_ang
            self.get_logger().info('Turning left')
        elif self.right == key:
            msg.angular.z = -1 * vel_ang
            self.get_logger().info('Turning right')
        elif self.up == key:
            msg.linear.x = vel_lin
            self.get_logger().info('Going forward')
        elif self.down == key:
            msg.linear.x = -1 * vel_lin
            self.get_logger().info('Going backward')
        else:
            print("Start moving by pressing one of keys: a d w s")

        self.publisher_.publish(msg)
        curses.nocbreak()
        stdscr.keypad(0)
        curses.echo()
        curses.endwin()


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
