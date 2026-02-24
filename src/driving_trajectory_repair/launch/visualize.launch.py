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
"""Inspection in rviz2. Play a bag alongside it:

    ros2 bag play recording_repaired

x_offset and y_offset are subtracted from every drawn position. Map coordinates
are UTM metres, which lose precision in the float32 transforms rviz uses, so
set them to a point near the scene.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _offsets():
    return {
        "frame_id": ParameterValue(
            LaunchConfiguration("frame_id"), value_type=str),
        "x_offset": ParameterValue(
            LaunchConfiguration("x_offset"), value_type=float),
        "y_offset": ParameterValue(
            LaunchConfiguration("y_offset"), value_type=float),
    }


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("frame_id", default_value="map"),
        DeclareLaunchArgument("x_offset", default_value="0.0"),
        DeclareLaunchArgument("y_offset", default_value="0.0"),
        DeclareLaunchArgument("run_estimation_visualizer",
                              default_value="true"),
        DeclareLaunchArgument("obstacles_topic", default_value="/obstacles"),
        DeclareLaunchArgument("repaired_topic",
                              default_value="/obstacles_modified"),
    ]

    # Predicted poses of tracks that perception dropped
    estimation = Node(
        package="driving_trajectory_repair",
        executable="estimation_visualizer",
        name="estimation_visualizer",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("run_estimation_visualizer")),
        parameters=[_offsets()],
        remappings=[("obstacles", LaunchConfiguration("obstacles_topic"))],
    )

    # Contents of a repaired or interpolated-only bag
    repaired = Node(
        package="driving_trajectory_repair",
        executable="repaired_trajectory_visualizer",
        name="repaired_trajectory_visualizer",
        output="screen",
        emulate_tty=True,
        parameters=[_offsets()],
        remappings=[
            ("obstacles_modified", LaunchConfiguration("repaired_topic"))],
    )

    return LaunchDescription(arguments + [estimation, repaired])
