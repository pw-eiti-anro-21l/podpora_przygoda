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
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from PyKDL import *
import yaml


class Ikin(Node):

	def __init__(self):
		super().__init__('ikin')
		self.values = loadDH()

		self.subscription = self.create_subscription(
			PoseStamped(),
			'/pose_stamped',
			self.listener_callback,10)
		self.subscription  

		self.last_x = 0
		self.last_y = 0
		self.last_z = 0


	def send_warning(self):
		self.ikin.get_logger().warn('Invalid position')


	def listener_callback(self, msg):

		pose = msg.pose.position.x
		pose_1 = msg.pose.position.y
		pose_2 = msg.pose.position.z
		
		

		dh_val = []

		for i, mark in enumerate(self.values.keys()):
			a, d, alpha, theta = self.values[mark]
			a, d, alpha, theta = float(a), float(d), float(alpha), float(theta)
			dh_val.append(d)


		pose =  pose + dh_val[0]
		pose_1 =  pose_1 + dh_val[1]
		pose_2 =  pose_2 + dh_val[2]

		qos_profile = QoSProfile(depth=10)
		self.joint_pub = self.create_publisher(JointState, '/joint_states', qos_profile)
		joint_state = JointState()
		now = self.get_clock().now()
		joint_state.header.stamp = now.to_msg()
		joint_state.name = ['base_link_1', 'link_1_2', 'link_2_3']
		print(pose)
		print(pose_1)
		print(pose_2)


		if(  (pose <= -0.1 or pose >= 0.1) or 
			(pose_1 <= -0.5 or pose_1 >= 1)  or
			(pose_2 <= -0.45 or pose_2 >= 1)):

			self.get_logger().warn("Invalid position")
			joint_state.position = [float(self.last_x), float(self.last_y), float(self.last_z)]
			
			self.joint_pub.publish(joint_state)


		else:
			joint_state.position = [pose, pose_1, pose_2]
			self.last_x = pose
			self.last_y = pose_1
			self.last_z = pose_2
			
			self.joint_pub.publish(joint_state)


def loadDH():

    with open(os.path.join(
        get_package_share_directory('lab5'),'DH.json'), 'r') as file:

        values = json.loads(file.read())

    return values



def main(args=None):

	rclpy.init(args=args)

	ikin = Ikin()
	rclpy.spin(ikin)

	ikin.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()

