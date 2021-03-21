from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
			package='turtlesim',
			executable='turtlesim_node',
			name='sim'
		),

		Node(
			package='lab1',
			executable='lab1_publisher',
			name='lab1_publisher',
			parameters=[{
				'up': 'w',
				'down': 's',
				'left': 'a',
				'right': 'd' 
			}]
		)

	])