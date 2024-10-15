#!/usr/bin/env python3
'''
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
'''
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from tmc_launch_ros_utils.tmc_launch_ros_utils import load_robot_description


def declare_arguments():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument('description_package',
                              default_value='hsre_description',
                              description='Description package with robot URDF/xacro files.'))
    declared_arguments.append(
        DeclareLaunchArgument('description_file',
                              default_value='hsre3p_whole_body.urdf.xacro',
                              description='URDF/XACRO description file with the robot base.'))
    declared_arguments.append(
        DeclareLaunchArgument('collision_config_file',
                              default_value='collision_pair_hsre3p.xml',
                              description='Robot collision config xml file.'))
    declared_arguments.append(
        DeclareLaunchArgument('validation_log_prefix',
                              description='File prefix of validation debug target.'))
    return declared_arguments


def load_robot_collision_config():
    config = load_robot_description(description_file_arg_name='collision_config_file')
    return {'robot_collision_pair': config['robot_description']}


def generate_launch_description():
    robot_description = load_robot_description()
    robot_collision_config = load_robot_collision_config()

    validation_offline = Node(package='tmc_collision_detecting_validator',
                              executable='validate_offline',
                              parameters=[robot_description, robot_collision_config],
                              #   arguments=['~/.ros/log/validation_20230628T173914_778619166'])
                              arguments=[LaunchConfiguration('validation_log_prefix')])

    return LaunchDescription(declare_arguments() + [validation_offline])
