import sys
from interpolation.srv import Interpolation
import rclpy
from rclpy.node import Node


class jint(Node):

    def __init__(self):
        super().__init__('jint_cli')
        self.cli = self.create_client(Interpolation, 'interpolation')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Interpolation.Request()

    def send_request(self):
        try:
            self.req.joint1_pose = float(sys.argv[1])
            self.req.joint2_pose = float(sys.argv[2])
            self.req.joint3_pose = float(sys.argv[3])
            self.req.type = sys.argv[5]
            self.future = self.cli.call_async(self.req)
        except ValueError:
            self.get_logger().info('ValueError while passing parameters ')
            self.req.joint1_pose = -1 #tu zrobic domyslne ustawienia
            self.req.joint2_pose = -1
            self.req.joint3_pose = -1
            self.req.move_time = f - 1
            self.req.type = sys.argv[5]
            self.future = self.client.call_async(self.req)



def main(args=None):
    rclpy.init(args=args)

    try:

        client = jint()
        client.send_request()
    except:
        print("Attempt Failed")
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
                    client.get_logger().info(response.output)
                    break
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()