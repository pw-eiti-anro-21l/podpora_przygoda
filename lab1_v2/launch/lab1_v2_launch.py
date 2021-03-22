from launch import LaunchDescription
from launch_ros.actions import Node
import pathlib

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
			prefix=['gnome-terminal --'],
			parameters=[{
				'up': 'w',
				'down': 's',
				'left': 'a',
				'right': 'd' 
			}]
		)

	])
