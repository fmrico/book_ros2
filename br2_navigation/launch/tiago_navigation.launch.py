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
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    launch_rmw_dir = get_package_share_directory('br2_navigation')
    nav2_dir = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')


    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(launch_rmw_dir, 'params', 'tiago_nav_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(launch_rmw_dir, 'maps', 'home.yaml'),
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    nav2_bringup_cmd_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'slam_launch.py')),
            condition=IfCondition(slam),
            launch_arguments={'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 
                                                       'localization_launch.py')),
            condition=IfCondition(PythonExpression(['not ', slam])),
            launch_arguments={'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_lifecycle_mgr': 'false'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_rmw_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_lifecycle_mgr': 'false',
                              'map_subscribe_transient_local': 'true'}.items()),
    ])

    ld = LaunchDescription()

    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
 
    ld.add_action(nav2_bringup_cmd_group)

    return ld
