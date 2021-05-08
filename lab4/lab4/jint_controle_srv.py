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
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import time
import math 
from interpolation.srv import Interpolation


class jint_controle_srv(Node):


    def __init__(self):
        self.theta= [1, 1, 0]
        super().__init__('jint_controle_srv')
        self.srv = self.create_service(Interpolation,'jint_control_srv', self.jint_callback)
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(JointState, 'joint_interpolate', qos_profile)
        self.start_position = [0,0,0]
        self.subscriber = self.create_subscription(JointState, 'joint_states',self.listener_callback, 10)
        self.in_action = False

    def listener_callback(self, msg):
        #pobieranie aktualnych położeń stawów
        if not self.in_action:
            self.start_position[0] = msg.position[0]
            self.start_position[1] = msg.position[1]
            self.start_position[2] = msg.position[2]

    def jint_callback(self, request, response):
        self.in_action = True

        self.interpolation(request)

        response.output = "Interpolation completed"
        self.in_action = False
        return response

    def interpolation(self, request):

        #pobieranie aktualnych położeń stawów

        sample_time = 0.1
        number_of_steps = math.floor(request.move_time / sample_time)
        joint_states = JointState()
        #te nazwy idk czy tak powinny byc
        joint_states.name = ['joint_base_1', 'joint_1_2', 'joint_2_3']
        start_position = self.start_position

        markerArray = MarkerArray()
        qos_profile = QoSProfile(depth=10)

        self.marker_pub = self.create_publisher(MarkerArray, '/marker', qos_profile)
        marker = Marker()
        marker.header.frame_id = "/base_link"

        marker.id = 0
        marker.action = Marker.DELETEALL
        markerArray.markers.append(marker)
        self.marker_pub.publish(markerArray)

        marker.type = marker.SPHERE
        marker.action = marker.ADD
        (marker.scale.x, marker.scale.y, marker.scale.z) = (0.05, 0.05, 0.05)
        (marker.color.a, marker.color.r, marker.color.g, marker.color.b) = (0.5, 1.0, 1.0, 1.0)
        (marker.pose.orientation.w, marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z) = (
            1.0, 1.0, 1.0, 1.0)

        t = request.move_time

        if request.type == 'polynomial':
            a0 = [start_position[0], start_position[1], start_position[2]]

            a1 = [0, 0, 0]

            a2 = []
            a2[0] = 3 * ((request.joint1_pose - start_position[0]) / t ** 2)
            a2[1] = 3 * ((request.joint2_pose - start_position[1]) / t ** 2)
            a2[3] = 3 * ((request.joint3_pose - start_position[2]) / t ** 2)

            a3 = []
            a3[0] = -2 * ((request.joint1_pose - start_position[0]) / t ** 3)
            a3[1] = -2 * ((request.joint2_pose - start_position[1]) / t ** 3)
            a3[3] = -2 * ((request.joint3_pose - start_position[2]) / t ** 3)

        for i in range(1, number_of_steps + 1):

            if request.type == 'linear':
                joint1_next = start_position[0] + ((request.joint1_pose - start_position[0]) / t) * sample_time * i
                joint2_next = start_position[1] + ((request.joint2_pose - start_position[1]) / t) * sample_time * i
                joint3_next = start_position[2] + ((request.joint3_pose - start_position[2]) / t) * sample_time * i

            if request.type == 'polynomial':
                joint1_next = a0[0] + a2[0] * (sample_time * i) ** 2 + a3[0] * (sample_time * i) ** 3
                joint2_next = a0[1] + a2[1] * (sample_time * i) ** 2 + a3[1] * (sample_time * i) ** 3
                joint3_next = a0[2] + a2[2] * (sample_time * i) ** 2 + a3[2] * (sample_time * i) ** 3

            ### tu trzeba dodac polozenie dokladniej, np +2, -3 do kazdego
            marker.pose.position.x = float(joint3_next)
            marker.pose.position.y = float(joint2_next)
            marker.pose.position.z = float(joint1_next)

            joint_states.position = [float(joint1_next), float(joint2_next), float(joint3_next)]
            self.publisher.publish(joint_states)
            time.sleep(sample_time)

            markerArray.markers.append(marker)

            id = 0
            for marker in markerArray.markers:
                marker.id = id
                id += 1

            self.marker_pub.publish(markerArray)

        self.start_positon = [joint1_next, joint2_next, joint3_next]


def main(args=None):
    rclpy.init(args=args)
    jint_node = jint_controle_srv()

    rclpy.spin(jint_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
