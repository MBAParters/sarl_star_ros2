# Copyright 2023 Clearpath Robotics, Inc.
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
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    leg_detect_dir = get_package_share_directory('leg_detect')
    sarl_star_ros2_dir = get_package_share_directory('sarl_star_ros2')
    laser_filters_dir = get_package_share_directory("laser_filters")

    laser_filters = Node(package='laser_filters', 
                         executable='scan_to_scan_filter_chain',
                         output='screen',
                         parameters=[
                         PathJoinSubstitution([
                            laser_filters_dir, "examples", "laserscan_filter.yaml",])]
                        )

    leg_detect = Node(package='leg_detect', 
                         executable='leg_detect',
                         name='leg_detect',
                         output='screen',
                         arguments=[os.path.join(leg_detect_dir, 'config', 'trained_leg_detector.yaml')])

    velocity_tracker = Node(package='people_velocity_tracking', 
                         executable='tracker',
                         name='velocity_tracker',
                         output='screen',
                         respawn=True)
    
    sarl_star_ros2 = Node(package='sarl_star_ros2', 
                         executable='sarl_star_node',
                         name='sarl_star_ros2',
                         output='screen')

    rviz2 = Node(package='rviz2', 
                 executable='rviz2',
                 name='rviz2',
                 output='screen',
                 arguments=['-d', os.path.join(sarl_star_ros2_dir, 'config', 'rviz.rviz')])

    ld = LaunchDescription()
    ld.add_action(laser_filters)
    ld.add_action(leg_detect)
    ld.add_action(velocity_tracker)
    # ld.add_action(sarl_star_ros2)
    ld.add_action(rviz2)
    return ld
