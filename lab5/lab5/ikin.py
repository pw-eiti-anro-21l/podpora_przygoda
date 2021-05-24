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
		self.values = readDHfile()

		# self.markerArray = MarkerArray()
		# qos_profile1 = QoSProfile(depth=10)
		# self.marker_pub = self.create_publisher(MarkerArray, '/marker', qos_profile1)
		# self.marker = Marker()
		# self.marker.header.frame_id = "base_link"
		# self.marker.id = 0
		# self.marker.action = Marker.DELETEALL
		# self.markerArray.markers.append(self.marker)
		# self.marker_pub.publish(self.markerArray)
		self.count = 0
		# self.MARKERS_MAX = 1000

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

		joint1 = msg.pose.position.z
		joint2 = msg.pose.position.y
		joint3 = msg.pose.position.x

		dh_val = []
		# self.marker.type = self.marker.SPHERE
		# self.marker.action = self.marker.ADD
		# (marker.scale.x,marker.scale.y, marker.scale.z) = (0.02, 0.02, 0.02)
		# self.marker.color.a = 1.0

		for i, mark in enumerate(self.values.keys()):
			a, d, alpha, theta = self.values[mark]
			a, d, alpha, theta = float(a), float(d), float(alpha), float(theta)
			dh_val.append(d)


		x =  joint3 + dh_val[0]
		y =  joint2 + dh_val[1]
		z =  joint1 + dh_val[2]

		qos_profile = QoSProfile(depth=10)
		self.joint_pub = self.create_publisher(JointState, '/joint_states', qos_profile)
		joint_state = JointState()
		now = self.get_clock().now()
		joint_state.header.stamp = now.to_msg()
		joint_state.name = ['base_link_1', 'link_1_2', 'link_2_3']
		print(x)
		print(y)
		print(z)


		if(  (x <= -0.5) or 
			(y <= 0)  or
			(z <= 0)):

			self.get_logger().warn("Invalid position")
			joint_state.position = [float(self.last_z), float(self.last_y), float(self.last_x)]
			
			self.joint_pub.publish(joint_state)

			# self.marker.pose.position.x = dh_val[0] + float(x)
			# self.marker.pose.position.y = dh_val[1] + float(y)
			# self.marker.pose.position.z = dh_val[2] + float(z)

			# if(self.count > self.MARKERS_MAX):
			# 	self.markerArray.markers.pop(0)
		
			# id = 0
			# for m in self.markerArray.markers:
			# 	m.id = id
			# 	id += 1
			# self.markerArray.markers.append(self.marker)
			self.count += 1

			#self.marker_pub.publish(self.markerArray)

		else:
			joint_state.position = [x, y, z]
			self.last_x = x
			self.last_y = y
			self.last_z = z
			
			self.joint_pub.publish(joint_state)

			# self.marker.pose.position.x = dh_val[0] + float(x)
			# self.marker.pose.position.y = dh_val[1] + float(y)
			# self.marker.pose.position.z = dh_val[2] + float(z)

			# if(self.count > self.MARKERS_MAX):
			# 	self.markerArray.markers.pop(0)
		
			# id = 0
			# for m in self.markerArray.markers:
			# 	m.id = id
			# 	id += 1
			# self.markerArray.markers.append(self.marker)
			self.count += 1

			#self.marker_pub.publish(self.markerArray)

def readDHfile():

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

