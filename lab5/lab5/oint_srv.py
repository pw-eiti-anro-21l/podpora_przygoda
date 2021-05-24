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
import json
import transforms3d

from interpolation.srv import Inverse


class oint_srv(Node):


    def __init__(self):
        super().__init__('oint_srv')
        self.values = loadDH()
        
        self.dh_val = []

        for i, mark in enumerate(self.values.keys()):
            a, d, alpha, theta = self.values[mark]
            a, d, alpha, theta = float(a), float(d), float(alpha), float(theta)
            self.dh_val.append(d)

        self.init_pose = [self.dh_val[0] + 0.03, self.dh_val[1] + 0.5, self.dh_val[2] + 0.45]

        self.count = 0
        self.MARKERS_MAX = 1000

        self.srv = self.create_service(Inverse, 'interpolation', self.interpolation_callback)  
        self.subscribtion = self.create_subscription(JointState, '/joint_states', self.initial_pose_callback, 10)

    def initial_pose_callback(self,msg):
        self.init_pose[0] = self.dh_val[0]  + msg.position[0]
        self.init_pose[1] = -self.dh_val[1] + msg.position[1]
        self.init_pose[2] = self.dh_val[2] + msg.position[2]

    def interpolation_callback(self, request, response):
        last_x = self.init_pose[0]
        last_y = self.init_pose[1]
        last_z = self.init_pose[2]

        self.get_logger().info('Incoming request')

        tp = 0.1 
        time_total = request.time_of_move
        steps = math.floor(time_total/tp) 
        
        # Eipsa
        if request.type == 'ellipse':

            while(True):

                for i in range(1,steps+1):

                    qos_profile = QoSProfile(depth=10)
                    pose_publisher = self.create_publisher(PoseStamped, '/pose_stamped', qos_profile)
                    poses = PoseStamped()
                    now = self.get_clock().now()
                    poses.header.stamp = ROSClock().now().to_msg()
                    poses.header.frame_id = "/base_link"

                    poses.pose.position.z = float(last_z) 
                    poses.pose.position.y = self.init_pose[1]  + request.param_a*math.cos(2 * math.pi * (1/request.time_of_move) * tp * i ) - request.param_a
                    poses.pose.position.x = self.init_pose[0] + request.param_b*math.sin(2 * math.pi * (1/request.time_of_move) * tp * i)
                    pose_publisher.publish(poses)

                    self.count += 1

                    time.sleep(tp)

        if request.type == 'spiral':
            while(True):

                for i in range(1,steps+1):

                    qos_profile = QoSProfile(depth=10)
                    pose_publisher = self.create_publisher(PoseStamped, '/pose_stamped', qos_profile)
                    poses = PoseStamped()
                    now = self.get_clock().now()
                    poses.header.stamp = ROSClock().now().to_msg()
                    poses.header.frame_id = "/base_link"

                    poses.pose.position.z = self.init_pose[2] + request.param_a*math.sin(2 * math.pi * (1/request.time_of_move) * tp * i)
                    poses.pose.position.y = self.init_pose[1]  + request.param_a*math.cos(2 * math.pi * (1/request.time_of_move) * tp * i )
                    poses.pose.position.x = self.init_pose[0] + request.param_b* ((tp/request.time_of_move)* i)
                    pose_publisher.publish(poses)

                    self.count += 1

                    time.sleep(tp)  

        response.confirmation = 'Finished interpolation'
        return response

def loadDH():

    with open(os.path.join(
        get_package_share_directory('lab5'),'DH.json'), 'r') as file:

        values = json.loads(file.read())

    return values



def main(args=None):
    rclpy.init(args=args)

    oint = oint_srv()

    rclpy.spin(oint)

    rclpy.shutdown()

if __name__ == '__main__':
    main()