# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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
# Author: Dr. Denis
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="schunk_lwa4p",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )

     # Initialize Arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")

    eds_file_path = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", "schunk_lwa4p", "Schunk_0_63.dcf"]
    )

    fake_slave_nodes = []
    for can_node_id in range(3, 9):
        fake_slave = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("canopen_fake_slaves"), "/launch", "/cia402_slave.launch.py"]
            ),
            launch_arguments={
                "node_id": f"{can_node_id}",
                "node_name": f"schunk_lwa4p_slave_{can_node_id-2}",
                "slave_config": eds_file_path,
            }.items(),
        )

        fake_slave_nodes.append(fake_slave)

    return LaunchDescription(
        declared_arguments
        + fake_slave_nodes
    )
