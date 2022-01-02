import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

  pkg_dir = get_package_share_directory('basics')
  param_file = os.path.join(pkg_dir, 'config', 'params.yaml')

  param_reader_cmd = Node(
    package='basics',
    executable='param_reader',
    parameters=[param_file],
    output='screen'
  )
  
  ld = LaunchDescription()
  ld.add_action(param_reader_cmd)

  return ld
