from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  
  return LaunchDescription([
    Node(
      package='basics',
      executable='publisher',
      output='screen'
    ),
    Node(
      package='basics',
      executable='subscriber_class',
      output='screen'
    )
  ])
