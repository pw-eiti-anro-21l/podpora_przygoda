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

    def send_request(self):
        try:
            self.req.joint1_goal = float(sys.argv[1])
            self.req.joint2_goal = float(sys.argv[2])
            self.req.joint3_goal = float(sys.argv[3])
            self.req.time_of_move = float(sys.argv[4])
            self.req.type = sys.argv[5]
            self.future = self.cli.call_async(self.req)
        except ValueError:
            self.get_logger().info('ValueError while passing parameters.')
            self.req.joint1_goal = -1
            self.req.joint2_goal = -1
            self.req.joint3_goal = -1
            self.req.time_of_move = -1
            self.req.type = sys.argv[5]
            self.future = self.client.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    try:

        client = jint()
        client.send_request()
    except:
        print("Request has been rejected.")
    else:

        while rclpy.ok():
            rclpy.spin_once(client)
            if client.future.done():
                try:
                    response = client.future.result()
                except Exception as e:
                    client.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    return
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
