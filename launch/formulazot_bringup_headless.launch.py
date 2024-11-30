"""
Skeleton Code based on work by Hongrui (Billy) Zheng from Upenn for ESE6150: F1Tenth Autonomous Racing Cars
https://github.com/f1tenth/f1tenth_lab2_template/tree/main

Copyright 2024 Ajay Shankar Sriram

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='autodrive_f1tenth',
            executable='autodrive_bridge',
            name='autodrive_bridge',
            emulate_tty=True,
            output='screen',
        ),
        Node(
            package='formulazot',
            executable='wall_follower',
            name='wall_follower',
        ),
    ])