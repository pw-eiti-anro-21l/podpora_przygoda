import sys
from interpolation.srv import OpInterpolation
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(OpInterpolation, 'interpolacja_op')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = OpInterpolation.Request()

    def send_request(self):
        try:
            # Położenia
            self.req.joint1_goal = float(sys.argv[1])
            self.req.joint2_goal= float(sys.argv[2])
            self.req.joint3_goal = float(sys.argv[3])

            # Zadane kąty RPY
            self.req.roll_goal = float(sys.argv[4])
            self.req.pitch_goal= float(sys.argv[5])
            self.req.yaw_goal = float(sys.argv[6])


            if(float(sys.argv[7])<=0):
                self.get_logger().info('Niepoprawna wartość czasu')
                raise ValueError("That is not a positive number!")
            else:
                self.req.time_of_move = float(sys.argv[7])

            if(str(sys.argv[8]) !='linear' and str(sys.argv[8]) !='polynomial' ):
                self.get_logger().info('Zły typ interpolacji ')
                raise ValueError("That is a wrong type!")
            else:
                self.req.type = (sys.argv[8])  
        except IndexError:
            print("Niepoprawna liczba parametrów")
            raise Exception()
        except ValueError:
            print("Błędne parametry")
            raise Exception()
            
        self.future = self.cli.call_async(self.req)



def main(args=None):
    rclpy.init(args=args)

    try:

        minimal_client = MinimalClientAsync()
        minimal_client.send_request()
    except:
        print("Anulowanie realizacji zapytania")
    else:


        while rclpy.ok():
            rclpy.spin_once(minimal_client)
            if minimal_client.future.done():
                try:
                    response = minimal_client.future.result()
                except Exception as e:
                    minimal_client.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    minimal_client.get_logger().info(
                        'Result of interpolation for positions: x = %d , y = %d , z = %d, roll = %d, pitch = %d, yaw = %d in time %d = %s' %
                        (minimal_client.req.joint1_goal, minimal_client.req.joint2_goal, minimal_client.req.joint3_goal,minimal_client.req.roll_goal, minimal_client.req.pitch_goal, minimal_client.req.yaw_goal, minimal_client.req.time_of_move, response.confirmation))
                    return
    finally:
        minimal_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
