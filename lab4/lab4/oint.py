import sys
from interpolation.srv import OintInterpolation
import rclpy
from rclpy.node import Node


class oint(Node):

    def __init__(self):
        super().__init__('oint')
        self.cli = self.create_client(OintInterpolation, 'oint_interpolacja')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = OintInterpolation.Request()


    def send_request(self):
        try:
            self.req.joint1_goal = float(sys.argv[1])
            self.req.joint2_goal= float(sys.argv[2])
            self.req.joint3_goal = float(sys.argv[3])

            self.req.roll_goal = float(sys.argv[4])
            self.req.pitch_goal= float(sys.argv[5])
            self.req.yaw_goal = float(sys.argv[6])
            self.req.time_of_move = float(sys.argv[7])
            self.req.type = (sys.argv[8]) 
        except ValueError:
            print("ValeuError while parsing")

            self.req.joint1_goal = -1
            self.req.joint2_goal= -1
            self.req.joint3_goal = -1

            self.req.roll_goal = -1
            self.req.pitch_goal= -1
            self.req.yaw_goal = -1
            self.req.time_of_move = 1
            self.req.type = (sys.argv[8]) 
            
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    try:

        client = oint()
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

