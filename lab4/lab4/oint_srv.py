from rclpy.node import Node
import rclpy
import math
from rclpy.clock import ROSClock
import time
from rclpy.qos import QoSProfile
from zad4_srv.srv import OintInterpolation
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Quaternion
import transforms3d

class oint_srv(Node):

	def __init__(self):
		super().__init__('oint_srv')
		self.srv = self.create_service(OintInterpolation,'oint_control_srv', self.oint_control_callback)
		qos_profile = QoSProfile(depth=10)
		self.publisher = self.create_publisher(PoseStamped, 'oint_interpolate', qos_profile)
		self.marker_publisher = self.create_publisher(MarkerArray, '/marker', qos_profile)
		self.start_position = [0,0,0]
		self.start_rotation = [0,0,0]
        
	def oint_control_callback(self, request, response):
		if request.move_time > 0:
			if request.interpolation_method=='lin' or request.interpolation_method=='lin_extended':
				self.linear_interpolation(request)
			elif request.interpolation_method=='pol' or request.interpolation_method=='pol_extended':
				self.polynomial_interpolation(request)
			else:
				self.linear_interpolation(request)
				response.output= "[!!!!!] Invalid interpolation method input. Methon changed to linear interpolation [!!!!!]"
			response.output = "Interpolation completed"
		else:
			self.get_logger().info('Time must be positive!!')

		return response



    # def listener_callback(self, msg):
    #     #pobieranie aktualnych położeń stawów
    #     if request.move_time > 0:
    #         self.start_position[0] = msg.position[0]
    #         self.start_position[1] = msg.position[1]
    #         self.start_position[2] = msg.position[2]
    #         self.start_rotation[0] = msg.position[3]
    #         self.start_rotation[1] = msg.position[4]
    #         self.start_rotation[2] = msg.position[5]
    #     else:
    # 		self.get_logger().info('Time must be positive!!')


    #     response.output = "Interpolation completed"
    #     return response




	def linear_interpolation(self, request):
		time_interval = 0.1
		number_of_steps = math.floor(request.move_time/time_interval)
		pose = PoseStamped()
		marker = Marker()
		marker_array = MarkerArray()
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 1.0
		marker.type = 1
		marker.action = 0
		marker.header.frame_id = "/base_link"


		for i in range(1, number_of_steps+1):
		    x_current = self.start_position[0]+((request.x_pose- self.start_position[0])/request.move_time)*time_interval*i
		    y_current = self.start_position[1]+((request.y_pose - self.start_position[1])/request.move_time)*time_interval*i
		    z_current = self.start_position[2]+((request.z_pose - self.start_position[2])/request.move_time)*time_interval*i


		    if request.interpolation_method=="lin_extended":
		    	roll_current=self.start_rotation[0]+((request.roll_pose- self.start_rotation[0])/request.move_time)*time_interval*i
		    	pitch_current=self.start_rotation[1]+((request.pitch_pose- self.start_rotation[1])/request.move_time)*time_interval*i
		    	yaw_current=self.start_rotation[2]+((request.yaw_pose- self.start_rotation[2])/request.move_time)*time_interval*i
		    	qua= transforms3d.euler.euler2quat(roll_current, pitch_current, yaw_current, axes='sxyz')
		    	qua_current = Quaternion(w=0.0, x=qua[0], y=qua[1], z=qua[2])
		    	# pose.pose.orientation = qua_current
		    	# marker.pose.orientation = qua_current
		    	


		    pose.header.frame_id = "base_link"
		    pose.pose.position.x = x_current
		    pose.pose.position.y = y_current
		    pose.pose.position.z = z_current
		    if request.interpolation_method=="lin_extended":
		    	pose.pose.orientation = qua_current

		    self.publisher.publish(pose)


		    marker.pose.position.x = x_current
		    marker.pose.position.y = y_current
		    marker.pose.position.z = z_current
		    if request.interpolation_method=="lin_extended":
		    	marker.pose.orientation = qua_current
		    
		    marker_array.markers.append(marker)
		    id=0
		    for marker in marker_array.markers:
		        marker.id = id
		        id += 1
		    self.marker_publisher.publish(marker_array)

		    time.sleep(time_interval)

		self.start_position=[x_current, y_current, z_current]
		if request.interpolation_method=="lin_extended":
			self.start_rotation=[roll_current, pitch_current, yaw_current]
		

	def polynomial_interpolation(self, request):
		time_interval = 0.1
		number_of_steps = math.floor(request.move_time/time_interval)
		pose = PoseStamped()
		marker = Marker()
		marker_array = MarkerArray()
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 1.0
		marker.type = 1
		marker.action = 0
		marker.header.frame_id = "/base_link"

		a0= [self.start_position[0], self.start_position[1], self.start_position[2]]
		a1= [0,0,0]
		a2= [3*(request.x_pose - self.start_position[0])/(request.move_time**2),
		3*(request.y_pose - self.start_position[1])/(request.move_time**2),
		3*(request.z_pose - self.start_position[2])/(request.move_time**2)
		]
		a3= [-2*(request.x_pose - self.start_position[0])/(request.move_time**3),
		-2*(request.y_pose - self.start_position[1])/(request.move_time**3),
		-2*(request.z_pose - self.start_position[2])/(request.move_time**3),
		]

		if request.interpolation_method=="pol_extended":
			a0r= [self.start_rotation[0], self.start_rotation[1], self.start_rotation[2]]
			a1r= [0,0,0]
			a2r= [3*(request.roll_pose - self.start_rotation[0])/(request.move_time**2),
			3*(request.pitch_pose - self.start_rotation[1])/(request.move_time**2),
			3*(request.yaw_pose - self.start_rotation[2])/(request.move_time**2)
			]
			a3r= [-2*(request.roll_pose - self.start_rotation[0])/(request.move_time**3),
			-2*(request.pitch_pose - self.start_rotation[1])/(request.move_time**3),
			-2*(request.yaw_pose - self.start_rotation[2])/(request.move_time**3),
			]


		for i in range(1, number_of_steps+1):
			x_current = a0[0]+ a1[0]*(time_interval*i) + a2[0]*((time_interval*i)**2)+ a3[0]*((time_interval*i)**3)
			y_current = a0[1]+ a1[1]*(time_interval*i) + a2[1]*((time_interval*i)**2)+ a3[1]*((time_interval*i)**3)
			z_current = a0[2]+ a1[2]*(time_interval*i) + a2[2]*((time_interval*i)**2)+ a3[2]*((time_interval*i)**3)


			if request.interpolation_method=="pol_extended":
				roll_current = a0r[0]+ a1r[0]*(time_interval*i) + a2r[0]*((time_interval*i)**2)+ a3r[0]*((time_interval*i)**3)
				pitch_current = a0r[1]+ a1r[1]*(time_interval*i) + a2r[1]*((time_interval*i)**2)+ a3r[1]*((time_interval*i)**3)
				yaw_current = a0r[2]+ a1r[2]*(time_interval*i) + a2r[2]*((time_interval*i)**2)+ a3r[2]*((time_interval*i)**3)
				qua= transforms3d.euler.euler2quat(roll_current, pitch_current, yaw_current, axes='sxyz')
				qua_current = Quaternion(w=0.0, x=qua[0], y=qua[1], z=qua[2])

			pose.header.frame_id = "base_link"
			pose.pose.position.x = x_current
			pose.pose.position.y = y_current
			pose.pose.position.z = z_current
			if request.interpolation_method=="pol_extended":
				pose.pose.orientation = qua_current

			self.publisher.publish(pose)


			marker.pose.position.x = x_current
			marker.pose.position.y = y_current
			marker.pose.position.z = z_current
			if request.interpolation_method=="pol_extended":
				marker.pose.orientation = qua_current

			marker_array.markers.append(marker)
			id=0
			for marker in marker_array.markers:
			    marker.id = id
			    id += 1
			self.marker_publisher.publish(marker_array)

			time.sleep(time_interval)

		self.start_position=[x_current, y_current, z_current]
		if request.interpolation_method=="pol_extended":
			self.start_rotation=[roll_current, pitch_current, yaw_current]






def main(args=None):
    rclpy.init(args=args)
    oint_node = oint_srv()

    rclpy.spin(oint_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()