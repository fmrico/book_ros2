# Copyright (c) 2021 Intelligent Robotics Lab (URJC)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription 

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution 

from launch_ros.actions import Node

import yaml

def generate_launch_description():
    mr2_kobuki_dir = get_package_share_directory('mr2_kobuki')

    params_file = os.path.join(mr2_kobuki_dir, 'config', 'kobuki_node_params.yaml')
    with open(params_file, 'r') as f:
        kobuki_params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']

    kobuki_cmd = Node(package='kobuki_node',
        executable='kobuki_ros_node',
        output='screen',
        parameters=[kobuki_params],
        )

    rplidar_cmd = Node(
        node_name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/lidar',
            'serial_baudrate': 115200,  # A1 / A2
            # 'serial_baudrate': 256000, # A3
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
    )

    tf_kobuki2laser_cmd = Node( package='tf2_ros', node_executable='static_transform_publisher', output='screen',
        arguments=['0.11', '0.0', '0.17',
                '3.1415', '0', '3.1415',
                'base_link',
                'laser'])

    laser_filter_cmd = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            PathJoinSubstitution([
                mr2_kobuki_dir,
                "config", "footprint_filter.yaml",
            ])],
        )

    ld = LaunchDescription()

    ld.add_action(kobuki_cmd)
    ld.add_action(rplidar_cmd)
    ld.add_action(tf_kobuki2laser_cmd)
    ld.add_action(laser_filter_cmd)
  
    return ld
