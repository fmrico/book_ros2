import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  
  return LaunchDescription([
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('basics'),
          'launch',
          'pub_sub_v2_launch.py'))
    )
  ])
