import rclpy
from rclpy.node import Node
import os
import mathutils
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from rclpy.clock import ROSClock
from sensor_msgs.msg import JointState
import time
import math  

from interpolation.srv import Interpolation


class jint_srv(Node):


    def __init__(self):
        super().__init__('jint_srv')
        self.srv = self.create_service(Interpolation, 'interpolacja', self.interpolation_callback)  

    def interpolation_callback(self, request, response):
                                               
        self.get_logger().info('Incoming request')

        if(request.type == 'polynomial'):
            self.polynomial(request)
        if(request.type == 'linear'):
            self.linear(request)

        response.confirmation = 'Interpolacja zakończona'
        return response

    def polynomial(self, request):
        start_positions = [0, 0, 0]
        sample_time = 0.1 
        total_time = request.time_of_move
        steps = math.floor(total_time/sample_time)
	
        a0 = [start_positions[0],start_positions[1],start_positions[2]]
            
        a2 = [0, 0, 0]
        a2[0] = 3*((request.joint1_goal - start_positions[0])/(request.time_of_move)**2)
        a2[1] = 3*((request.joint2_goal - start_positions[1])/(request.time_of_move)**2)
        a2[2] = 3*((request.joint3_goal - start_positions[2])/(request.time_of_move)**2)
            
            
        a3 = [0, 0, 0]
        a3[0] = -2*((request.joint1_goal - start_positions[0])/(request.time_of_move)**3)
        a3[1] = -2*((request.joint2_goal - start_positions[1])/(request.time_of_move)**3)
        a3[2] = -2*((request.joint3_goal - start_positions[2])/(request.time_of_move)**3)

        for i in range(1,steps+1):
            
            qos_profile1 = QoSProfile(depth=10)
            self.joint_pub = self.create_publisher(JointState, '/joint_states', qos_profile1)
            joint_state = JointState()
            now = self.get_clock().now()
            joint_state.header.stamp = now.to_msg()
            joint_state.name = ['base_link_1', 'link_1_2', 'link_2_3']

            joint1_next = a0[0] + a2[0]*(sample_time*i)**2 + a3[0]*(sample_time*i)**3 
            joint2_next = a0[1] + a2[1]*(sample_time*i)**2 + a3[1]*(sample_time*i)**3 
            joint3_next = a0[2] + a2[2]*(sample_time*i)**2 + a3[2]*(sample_time*i)**3 
            
            joint_state.position = [float(joint1_next), float(joint2_next), float(joint3_next)]

            self.joint_pub.publish(joint_state)
            time.sleep(sample_time)
	
	
    def linear(self, request):

        start_positions = [0, 0, 0]
        sample_time = 0.1 
        total_time = request.time_of_move
        steps = math.floor(total_time/sample_time)

        for i in range(1,steps+1):
            
            qos_profile1 = QoSProfile(depth=10)
            self.joint_pub = self.create_publisher(JointState, '/joint_states', qos_profile1)
            joint_state = JointState()
            now = self.get_clock().now()
            joint_state.header.stamp = now.to_msg()
            joint_state.name = ['base_link_1', 'link_1_2', 'link_2_3']

            joint1_next = start_positions[0] + ((request.joint1_goal - start_positions[0])/request.time_of_move)*sample_time*i
            joint2_next = start_positions[1] + ((request.joint2_goal - start_positions[1])/request.time_of_move)*sample_time*i
            joint3_next = start_positions[2] + ((request.joint3_goal - start_positions[2])/request.time_of_move)*sample_time*i            
            joint_state.position = [float(joint1_next), float(joint2_next), float(joint3_next)]

            self.joint_pub.publish(joint_state)
            time.sleep(sample_time)


def main(args=None):
    rclpy.init(args=args)

    service = jint_srv()

    rclpy.spin(service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
