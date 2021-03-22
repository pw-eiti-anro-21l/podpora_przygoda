import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

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
        left = self.get_parameter('left').get_parameter_value().string_value
        right = self.get_parameter('right').get_parameter_value().string_value
        up = self.get_parameter('up').get_parameter_value().string_value
        down = self.get_parameter('down').get_parameter_value().string_value
        msg = Twist()
        vel_ang = 1.0
        vel_lin = 1.0

        self.key_mapping = {
            up: [0, vel_lin],
            down: [0, -1*vel_lin],
            left: [vel_ang, 0],
            right: [-1*vel_ang, 0]
        }
        user_input = getch.getch()
        if len(user_input) == 0 or not self.correct_key(user_input):
            return

        msg.angular.z = float(self.key_mapping[user_input[0]][0])
        msg.linear.x = float(self.key_mapping[user_input[0]][1])

        self.publisher_.publish(msg)
        self.left = left
        self.right = right
        self.up = up
        self.down = down

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
