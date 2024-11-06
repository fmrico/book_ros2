# Copyright 2021 Intelligent Robotics Lab
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
from launch_ros.actions import Node


def generate_launch_description():

    params_file = os.path.join(
        get_package_share_directory('br2_tracking'),
        'config',
        'detector.yaml'
        )

    object_tracker_cmd = Node(
        package='br2_tracking',
        executable='object_tracker',
        parameters=[{
          'use_sim_time': True
        }, params_file],
        remappings=[
          ('input_image', '/head_front_camera/image'),
          ('joint_state', '/head_controller/controller_state'),
          ('joint_command', '/head_controller/joint_trajectory')
        ],
        output='screen'
    )

    ld = LaunchDescription()

    # Add any actions
    ld.add_action(object_tracker_cmd)

    return ld
