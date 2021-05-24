import sys
from interpolation.srv import Inverse
import rclpy
from rclpy.node import Node


class oint(Node):

    def __init__(self):
        super().__init__('oint')
        self.cli = self.create_client(Inverse, 'interpolation')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Inverse.Request()


    def send_request(self):
        try:
            self.req.initial_x = float(sys.argv[1])
            self.req.initial_y = float(sys.argv[2])
            self.req.initial_z = float(sys.argv[3])
            self.req.time_of_move = float(sys.argv[4])

            self.req.type = sys.argv[5] 

            self.req.param_a= float(sys.argv[6])
            self.req.param_b = float(sys.argv[7])
            

        except ValueError:
            print("ValeuError while parsing")

            self.req.initial_x = 1
            self.req.initial_y = 1
            self.req.initial_z = 1
            self.req.time_of_move = 1

            self.req.type = (sys.argv[5]) 

            self.req.param_a = 1
            self.req.param_b = 1
            
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
                    client.get_logger().info(
                        'Interpolation completed')
                    return
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

