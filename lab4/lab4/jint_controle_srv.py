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
        super().__init__('jint_controle_srv')
        self.srv = self.create_service(Interpolation, 'interpolation', self.jint_callback)

    def jint_callback(self, request, response):
        start_position = [0, 0, 0]
        #pobieranie aktualnych położeń stawów
       
        sample_time = 0.1
        number_of_steps = math.floor(request.move_time / sample_time)

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

            a2 = [0,0,0]
            a2[0] = 3 * ((request.joint1_pose - start_position[0]) / t ** 2)
            a2[1] = 3 * ((request.joint2_pose - start_position[1]) / t ** 2)
            a2[2] = 3 * ((request.joint3_pose - start_position[2]) / t ** 2)

            a3 = [0,0,0]
            a3[0] = -2 * ((request.joint1_pose - start_position[0]) / t ** 3)
            a3[1] = -2 * ((request.joint2_pose - start_position[1]) / t ** 3)
            a3[2] = -2 * ((request.joint3_pose - start_position[2]) / t ** 3)

        for i in range(1, number_of_steps + 1):
            qos_profile2 = QoSProfile(depth=10)
            self.joint_pub = self.create_publisher(JointState, '/joint_states', qos_profile2)
            joint_states = JointState()
            joint_states.name = ['link_base_1', 'link_1_2', 'link_2_4']

            now = self.get_clock().now()
            joint_states.header.stamp = now.to_msg()

            joint1_next = start_position[0] + ((request.joint1_pose - start_position[0]) / t) * sample_time * i
            joint2_next = start_position[1] + ((request.joint2_pose - start_position[1]) / t) * sample_time * i
            joint3_next = start_position[2] + ((request.joint3_pose - start_position[2]) / t) * sample_time * i

            if request.type == 'polynomial':
                joint1_next = a0[0] + a2[0] * (sample_time * i) * 2 + a3[0] * (sample_time * i) * 3
                joint2_next = a0[1] + a2[1] * (sample_time * i) * 2 + a3[1] * (sample_time * i) * 3
                joint3_next = a0[2] + a2[2] * (sample_time * i) * 2 + a3[2] * (sample_time * i) * 3

            ### tu trzeba dodac polozenie dokladniej, np +2, -3 do kazdego
            marker.pose.position.x = float(joint3_next)
            marker.pose.position.y = float(joint2_next)
            marker.pose.position.z = float(joint1_next)

            joint_states.position = [float(joint1_next), float(joint2_next), float(joint3_next)]
            start_position = [joint1_next, joint2_next, joint3_next]

            markerArray.markers.append(marker)

            id = 0
            for marker in markerArray.markers:
                marker.id = id
                id += 1

            self.marker_pub.publish(markerArray)
            self.joint_pub.publish(joint_states)
            time.sleep(sample_time)

        response.output = "Interpolation completed"
        return response


def main(args=None):
    rclpy.init(args=args)
    jint_node = jint_controle_srv()

    rclpy.spin(jint_node)

    rclpy.shutdown()


if _name_ == '_main_':
    main()
