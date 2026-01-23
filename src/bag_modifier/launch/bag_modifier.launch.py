# Copyright 2026 Korea Electronics Technology Institute (KETI)
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
"""Repairs one recorded bag.

This is a batch tool, not a long-running node: it exits once the output bags
have been written.

    ros2 launch bag_modifier bag_modifier.launch.py bag:=/path/to/recording
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("bag_modifier"), "config",
        "bag_modifier.yaml")

    arguments = [
        DeclareLaunchArgument(
            "bag", description="Directory of the recording. Never modified."),
        DeclareLaunchArgument(
            "suffix", default_value="_repaired",
            description="Appended to the output bag names."),
        DeclareLaunchArgument("config", default_value=default_config),
    ]

    repair = Node(
        package="bag_modifier",
        executable="bag_modifier",
        name="bag_modifier",
        output="screen",
        emulate_tty=True,
        arguments=[LaunchConfiguration("bag"), LaunchConfiguration("suffix")],
        parameters=[LaunchConfiguration("config")],
    )

    return LaunchDescription(arguments + [repair])
