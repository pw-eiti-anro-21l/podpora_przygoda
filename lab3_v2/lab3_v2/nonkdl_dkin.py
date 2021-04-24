from math import sin, cos, pi
import os
import rclpy
import mathutils
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster, TransformStamped
import json
from rclpy.clock import ROSClock
import time


class NONKDL_DKIN(Node):

    def __init__(self):
        super().__init__('NONKDL_DKIN')

        self.subscription = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)
        self.subscription

    def listener_callback(self, msg):
        with open('DH.json', 'r') as file:
            params = json.loads(file.read())

        i = 1
        T = []
        for i,key in enumerate(params.keys()):
            a, d, alpha, theta = params[key]
            a = float(a)
            d = float(d)
            alpha = float(alpha)
            theta = float(theta)

            rot_x = mathutils.Matrix.Rotation(alpha, 4, 'X')
            trans_x = mathutils.Matrix.Translation((a, 0, 0))
            rot_z = mathutils.Matrix.Rotation(theta, 4, 'Z')
            trans_z = mathutils.Matrix.Translation((0, 0, d + msg.position[i]))

            dh = rot_x @ trans_x @ rot_z @ trans_z
            T.append(dh)

        T_k = T[0] @ T[1] @ T[2]

        xyz = T_k.to_translation()
        rpy = T_k.to_euler()
        qua = rpy.to_quaternion()

        qos_profile = QoSProfile(depth=10)
        pose_publisher = self.create_publisher(PoseStamped, '/pose_stamped', qos_profile)

        pose_stamped = PoseStamped()
        now = self.get_clock().now()
        pose_stamped.header.stamp = ROSClock().now().to_msg()
        pose_stamped.header.frame_id = "base_link"

        pose_stamped.pose.position.x = xyz[0] 
        pose_stamped.pose.position.y = xyz[1]
        pose_stamped.pose.position.z = xyz[2]
        pose_stamped.pose.orientation = Quaternion(w=qua[0], x=qua[1], y=qua[2], z=qua[3])
        pose_publisher.publish(pose_stamped)


def main(args=None):
    rclpy.init(args=args)
    nonkdl = NONKDL_DKIN()
    rclpy.spin(nonkdl)
    nonkdl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
