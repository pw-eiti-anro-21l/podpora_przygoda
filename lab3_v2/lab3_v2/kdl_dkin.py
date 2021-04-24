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