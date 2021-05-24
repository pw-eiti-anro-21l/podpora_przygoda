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

    
        # markerArray = MarkerArray()
        # qos_profile = QoSProfile(depth=10)
        # self.marker_pub = self.create_publisher(MarkerArray, '/marker_pose', qos_profile)
        # marker = Marker()
        # marker.header.frame_id = "/base_link"
        # marker.id = 0
        # marker.action = Marker.DELETEALL
        # markerArray.markers.append(marker)

        # self.marker_pub.publish(markerArray)

        # marker.type = marker.SPHERE
        # marker.action = marker.ADD
        # (marker.scale.x,marker.scale.y, marker.scale.z) = (0.02, 0.02, 0.02)
        # (marker.color.a, marker.color.r, marker.color.g, marker.color.b) = (1.0, 1.0, 0.0, 1.0)
        # (marker.pose.orientation.w, marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z) = (1.0, 1.0, 1.0, 1.0)

        
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

                    # if(self.count > self.MARKERS_MAX):
                        
                    #     markerArray.markers.pop(0)

                    # marker.pose.position.x = poses.pose.position.x
                    # marker.pose.position.y = poses.pose.position.z
                    # marker.pose.position.z = poses.pose.position.y
                    # markerArray.markers.append(marker)
                    # id = 0
                    # for m in markerArray.markers:
                    #     m.id = id
                    #     id += 1
                    self.count += 1
                    # self.marker_pub.publish(markerArray)


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