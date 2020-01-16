# Copyright 2019 Intelligent Robotics Lab
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
#
# Author: David Vargas Frutos <david.vargas@urjc.es>

import os
import sys
import launch

import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle

import lifecycle_msgs.msg

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import EmitEvent
from launch.actions import SetEnvironmentVariable
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    params_path = os.path.join(get_package_share_directory('parameters_test'), 'config', 'vicon_driver_params.yaml')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    print('')
    print('params_path: ', params_path)
    print('')

    main_node = Node(
        node_name='param_test_node',
        package='parameters_test',
        node_executable='parameters_test_main',
        output='screen',
        parameters=[params_path],
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    #ld.add_action(declare_params_file_cmd)
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(main_node)

    return ld
