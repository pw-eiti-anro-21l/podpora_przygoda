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
from PyKDL import *
import yaml


class KDL_DKIN(Node):

    def __init__(self):
        super().__init__('KDL_DKIN')

        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):

        with open('urdf.yaml', 'r') as file:  # na razie tutaj bez sciezki
            params = yaml.load(file, Loader=yaml.FullLoader)

        params_list = []
        params_float = []
        params_list.append(params['i1']['joint_xyz'])
        params_list.append(params['i1']['joint_rpy'])
        params_list.append(params['i2']['joint_xyz'])
        params_list.append(params['i2']['joint_rpy'])
        params_list.append(params['i3']['joint_xyz'])
        params_list.append(params['i3']['joint_rpy'])

        for param in params_list:
            params_split = param.split()
            params_float.append([float(param_split) for param_split in params_split])

        chain = self.create_chain(params_float)

        joint_positions = JntArray(3)

        joint_positions[0] = msg.position[0]
        joint_positions[1] = msg.position[1]
        joint_positions[2] = msg.position[2]

        fk = ChainFkSolverPos_recursive(chain)
        frame = Frame()
        fk.JntToCart(joint_positions, frame)

        t_offset = Vector(0, 0, 0)
        xyz = frame.p + t_offset
        qua = frame.M.GetQuaternion()

        qos_profile = QoSProfile(depth=10)
        pose_publisher = self.create_publisher(PoseStamped, '/pose_stamped', qos_profile)

        pose_stamped = PoseStamped()
        now = self.get_clock().now()
        pose_stamped.header.stamp = ROSClock().now().to_msg()
        pose_stamped.header.frame_id = "base_link"

        pose_stamped.pose.position.x = xyz[0]
        pose_stamped.pose.position.y = xyz[1]
        pose_stamped.pose.position.z = xyz[2]
        pose_stamped.pose.orientation = Quaternion(w=float(qua[0]), x=float(qua[1]), y=float(qua[2]), z=float(qua[3]))

        pose_publisher.publish(pose_stamped)

    def create_chain(self, params_float):
        chain = Chain()

        link_base_1 = Joint(Joint.TransZ)
        frame_base_1 = Frame(Rotation.RPY(
            params_float[1][0],
            params_float[1][1],
            params_float[1][2]), Vector(
            params_float[0][0],
            params_float[0][1],
            params_float[0][2]))

        segment_base_1 = Segment(link_base_1, frame_base_1)

        link_1_2 = Joint(Joint.RotZ)
        frame_1_2 = Frame(Rotation.RPY(
            params_float[3][0],
            params_float[3][1],
            params_float[3][2]), Vector(
            params_float[2][0],
            params_float[2][1],
            params_float[2][2]))

        segment_1_2 = Segment(link_1_2, frame_1_2)

        link_2_3 = Joint(Joint.RotZ)
        frame_2_3 = Frame(Rotation.RPY(
            params_float[5][0],
            params_float[5][1],
            params_float[5][2]), Vector(
            params_float[4][0],
            params_float[4][1],
            params_float[4][2]))

        segment_2_3 = Segment(link_2_3, frame_2_3)

        chain.addSegment(segment_base_1)
        chain.addSegment(segment_1_2)
        chain.addSegment(segment_2_3)

        return chain


def main(args=None):
    rclpy.init(args=args)
    kdl = KDL_DKIN()
    rclpy.spin(kdl)
    kdl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
