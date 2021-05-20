
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
import transforms3d

from interpolation.srv import OintInterpolation


class oint_srv(Node):


    def __init__(self):
        super().__init__('oint_srv')
        self.srv = self.create_service(OintInterpolation, 'oint_interpolacja', self.oint_callback)  

    def oint_callback(self, request, response):

        self.get_logger().info('Processing request')

        markerArray = MarkerArray()
        qos_profile = QoSProfile(depth=10)

        self.marker_pub = self.create_publisher(MarkerArray, '/marker', qos_profile)
        marker = Marker()
        
        marker.header.frame_id = "/base_frame"
        marker.id = 0
        marker.action = Marker.DELETEALL
        markerArray.markers.append(marker)

        self.marker_pub.publish(markerArray)

        marker.type = marker.SPHERE
        marker.action = marker.ADD
        (marker.scale.x,marker.scale.y, marker.scale.z) = (0.02, 0.02, 0.02)
        (marker.color.a, marker.color.r, marker.color.g, marker.color.b) = (1.0, 1.0, 0.0, 1.0)
        (marker.pose.orientation.w, marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z) = (1.0, 1.0, 1.0, 1.0)

        st = 0.1 
        total_time = request.time_of_move
        steps = math.floor(total_time/st)
        start_positions = [0, 0, 0]
        start_orientation = [0, 0, 0]
        if(request.type == 'polynomial'):

            a0 = [start_positions[0], start_positions[1], start_positions[2]]

            a2 = [0, 0, 0]
            a2[0] = 3 * ((request.joint1_goal - start_positions[0]) / (request.time_of_move) ** 2)
            a2[1] = 3 * ((request.joint2_goal - start_positions[1]) / (request.time_of_move) ** 2)
            a2[2] = 3 * ((request.joint3_goal - start_positions[2]) / (request.time_of_move) ** 2)

            a3 = [0, 0, 0]
            a3[0] = -2 * ((request.joint1_goal - start_positions[0]) / (request.time_of_move) ** 3)
            a3[1] = -2 * ((request.joint2_goal - start_positions[1]) / (request.time_of_move) ** 3)
            a3[2] = -2 * ((request.joint3_goal - start_positions[2]) / (request.time_of_move) ** 3)

            a0_1 = [start_orientation[0], start_orientation[1], start_orientation[2]]

            a2_1 = [0, 0, 0]
            a2_1[0] = 3 * ((request.roll_goal - start_orientation[0]) / (request.time_of_move) ** 2)
            a2_1[1] = 3 * ((request.pitch_goal - start_orientation[1]) / (request.time_of_move) ** 2)
            a2_1[2] = 3 * ((request.yaw_goal - start_orientation[2]) / (request.time_of_move) ** 2)

            a3_1 = [0, 0, 0]
            a3_1[0] = -2 * ((request.roll_goal - start_orientation[0]) / (request.time_of_move) ** 3)
            a3_1[1] = -2 * ((request.pitch_goal - start_orientation[1]) / (request.time_of_move) ** 3)
            a3_1[2] = -2 * ((request.yaw_goal - start_orientation[2]) / (request.time_of_move) ** 3)

        for i in range(1,steps+1):

            qos_profile = QoSProfile(depth=10)
            pose_publisher = self.create_publisher(PoseStamped, '/pose_oint', qos_profile)
            poses = PoseStamped()
            now = self.get_clock().now()
            poses.header.stamp = ROSClock().now().to_msg()
            poses.header.frame_id = "map"

            pose_publisher_rpy = self.create_publisher(PoseStamped, '/orientation', qos_profile)
            poses_rpy = PoseStamped()
            poses_rpy.header.stamp = ROSClock().now().to_msg()
            poses_rpy.header.frame_id = "map"

            if(request.type == 'linear'):

                poses.pose.position.x = start_positions[0] + ((request.joint1_goal - start_positions[0])/request.time_of_move)*st*i
                poses.pose.position.y = start_positions[1] + ((request.joint2_goal - start_positions[1])/request.time_of_move)*st*i
                poses.pose.position.z = start_positions[2] + ((request.joint3_goal - start_positions[2])/request.time_of_move)*st*i

                frame_roll = start_orientation[0] + ((request.roll_goal - start_orientation[0])/request.time_of_move)*st*i
                frame_pitch = start_orientation[1] + ((request.pitch_goal - start_orientation[1])/request.time_of_move)*st*i
                frame_yaw = start_orientation[2] + ((request.yaw_goal - start_orientation[2])/request.time_of_move)*st*i

                
            if(request.type == 'polynomial'):

                poses.pose.position.x = a0[0] + a2[0]*(st*i)**2 + a3[0]*(st*i)**3 
                poses.pose.position.y = a0[1] + a2[1]*(st*i)**2 + a3[1]*(st*i)**3 
                poses.pose.position.z = a0[2] + a2[2]*(st*i)**2 + a3[2]*(st*i)**3 

                frame_roll = a0_1[0] + a2_1[0]*(st*i)**2 + a3_1[0]*(st*i)**3 
                frame_pitch = a0_1[1] + a2_1[1]*(st*i)**2 + a3_1[1]*(st*i)**3 
                frame_yaw = a0_1[2] + a2_1[2]*(st*i)**2 + a3_1[2]*(st*i)**3 

            poses_rpy.pose.position.x = poses.pose.position.x
            poses_rpy.pose.position.y = poses.pose.position.y
            poses_rpy.pose.position.z = poses.pose.position.z
            poses_rpy.pose.orientation = Quaternion(w= 0.0, x=frame_roll, y=frame_pitch, z=frame_yaw)

            pose_publisher_rpy.publish(poses_rpy)

            frame_roll = frame_roll*math.pi/180
            frame_pitch = frame_pitch*math.pi/180
            frame_yaw = frame_yaw*math.pi/180

            quaternion = transforms3d.euler.euler2quat(frame_roll, frame_pitch, frame_yaw, axes='sxyz')
            poses.pose.orientation = Quaternion(w=quaternion[0], x=quaternion[1], y=quaternion[2], z=quaternion[3])

            marker.pose.position.x = poses.pose.position.x
            marker.pose.position.y = poses.pose.position.y
            marker.pose.position.z = poses.pose.position.z

            
            markerArray.markers.append(marker)

            id = 0
            for m in markerArray.markers:
                m.id = id
                id += 1

            self.marker_pub.publish(markerArray)
            pose_publisher.publish(poses)
            time.sleep(st)

        response.confirmation = 'Interpolation completed.'
        return response


def main(args=None):
    rclpy.init(args=args)

    service = oint_srv()

    rclpy.spin(service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()