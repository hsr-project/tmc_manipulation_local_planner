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
import argparse
from collections.abc import Callable
import fcntl
import math
import os
from pathlib import Path
import sys
import termios
import time
from typing import Any
from typing import TypeAlias

from geometry_msgs.msg import (
    Pose,
    Transform,
    TransformStamped,
)
from moveit_msgs.msg import (
    DisplayTrajectory,
    RobotTrajectory,
)
import rclpy
import rclpy.node
import rclpy.publisher

from rosbags.rosbag2 import Reader
from rosbags.typesys import (
    get_types_from_msg,
    get_typestore,
    Stores,
)
import rosbags.typesys.store

from sensor_msgs.msg import JointState
import tf2_ros
from tmc_planning_msgs.msg import (
    Constraints,
    TsrLinkConstraint,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
    MultiDOFJointTrajectoryPoint,
)
from visualization_msgs.msg import Marker, MarkerArray


def get_key():
    # https://qiita.com/pukin/items/3b791b8b759dd704f765
    fno = sys.stdin.fileno()

    attr_old = termios.tcgetattr(fno)

    attr = termios.tcgetattr(fno)
    attr[3] = attr[3] & ~termios.ECHO & ~termios.ICANON
    termios.tcsetattr(fno, termios.TCSADRAIN, attr)

    fcntl_old = fcntl.fcntl(fno, fcntl.F_GETFL)
    fcntl.fcntl(fno, fcntl.F_SETFL, fcntl_old | os.O_NONBLOCK)

    chr = 0

    try:
        c = sys.stdin.read(1)
        if len(c):
            while len(c):
                chr = (chr << 8) + ord(c)
                c = sys.stdin.read(1)
    finally:
        fcntl.fcntl(fno, fcntl.F_SETFL, fcntl_old)
        termios.tcsetattr(fno, termios.TCSANOW, attr_old)

    return chr


# Is this okay?
DisplayTrajectoryRosBag: TypeAlias = 'rosbags.usertypes.moveit_msgs__msg__DisplayTrajectory'
RobotLocalGoalRosBag: TypeAlias = 'rosbags.usertypes.tmc_planning_msgs__msg__RobotLocalGoal'
JointStateRosBag: TypeAlias = rosbags.typesys.stores.ros2_dashing.sensor_msgs__msg__JointState
JointTrajectoryControllerStateRosBag: TypeAlias = 'rosbags.usertypes.control_msgs__msg__JointTrajectoryControllerState'
MarkerArrayRosBag: TypeAlias = rosbags.typesys.stores.ros2_dashing.visualization_msgs__msg__MarkerArray
PoseRosBag: TypeAlias = rosbags.typesys.stores.ros2_dashing.geometry_msgs__msg__Pose
TsrLinkConstraintRosBag: TypeAlias = 'rosbags.usertypes.tmc_planning_msgs__msg__TsrLinkConstraint'


def publish_joint_state(pub: rclpy.publisher.Publisher, joint_state_rosbag: JointStateRosBag) -> None:
    if joint_state_rosbag is None:
        return
    msg = JointState()
    msg.name = joint_state_rosbag.name
    msg.position = joint_state_rosbag.position.tolist()
    pub.publish(msg)


def broadcast_base_state(broadcaster: tf2_ros.StaticTransformBroadcaster,
                         node: rclpy.node.Node,
                         controller_state: JointTrajectoryControllerStateRosBag) -> None:
    if controller_state is None:
        return
    transform = TransformStamped()
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.header.frame_id = 'odom'
    transform.child_frame_id = 'base_link'
    transform.transform.translation.x = controller_state.actual.positions[0]
    transform.transform.translation.y = controller_state.actual.positions[1]
    transform.transform.rotation.z = math.sin(controller_state.actual.positions[2] / 2.0)
    transform.transform.rotation.w = math.cos(controller_state.actual.positions[2] / 2.0)
    broadcaster.sendTransform(transform)


def convert_to_pose_msg(pose_rosbag: PoseRosBag) -> Pose:
    msg = Pose()
    msg.position.x = pose_rosbag.position.x
    msg.position.y = pose_rosbag.position.y
    msg.position.z = pose_rosbag.position.z
    msg.orientation.x = pose_rosbag.orientation.x
    msg.orientation.y = pose_rosbag.orientation.y
    msg.orientation.z = pose_rosbag.orientation.z
    msg.orientation.w = pose_rosbag.orientation.w
    return msg


def publish_marker_array(pub: rclpy.publisher.Publisher, marker_array_rosbag: MarkerArrayRosBag) -> None:
    if marker_array_rosbag is None:
        return
    msg = MarkerArray()
    for marker_rosbag in marker_array_rosbag.markers:
        marker_msg = Marker()
        marker_msg.header.frame_id = marker_rosbag.header.frame_id
        marker_msg.ns = marker_rosbag.ns
        marker_msg.id = marker_rosbag.id
        marker_msg.type = marker_rosbag.type
        marker_msg.action = marker_rosbag.action
        marker_msg.pose = convert_to_pose_msg(marker_rosbag.pose)
        marker_msg.scale.x = marker_rosbag.scale.x
        marker_msg.scale.y = marker_rosbag.scale.y
        marker_msg.scale.z = marker_rosbag.scale.z
        marker_msg.color.r = marker_rosbag.color.r
        marker_msg.color.g = marker_rosbag.color.g
        marker_msg.color.b = marker_rosbag.color.b
        marker_msg.color.a = marker_rosbag.color.a
        msg.markers.append(marker_msg)
    pub.publish(msg)


def convert_to_tsr_link_constraint_msg(constraint_rosbag: TsrLinkConstraintRosBag) -> TsrLinkConstraint:
    tsr_constraint = TsrLinkConstraint()
    tsr_constraint.header.frame_id = constraint_rosbag.header.frame_id
    tsr_constraint.tsr.end_frame_id = constraint_rosbag.tsr.end_frame_id
    tsr_constraint.tsr.origin_to_tsr = convert_to_pose_msg(constraint_rosbag.tsr.origin_to_tsr)
    tsr_constraint.tsr.tsr_to_end = convert_to_pose_msg(constraint_rosbag.tsr.tsr_to_end)
    tsr_constraint.tsr.min_bounds = constraint_rosbag.tsr.min_bounds.tolist()
    tsr_constraint.tsr.max_bounds = constraint_rosbag.tsr.max_bounds.tolist()
    return tsr_constraint


def publish_constraints_from_goal(pub: rclpy.publisher.Publisher, goal_rosbag: RobotLocalGoalRosBag) -> None:
    if goal_rosbag is None:
        return
    msg = Constraints()
    msg.hard_link_constraints = [
        convert_to_tsr_link_constraint_msg(x) for x in goal_rosbag.constraints.hard_link_constraints]
    msg.soft_link_constraints = [
        convert_to_tsr_link_constraint_msg(x) for x in goal_rosbag.constraints.soft_link_constraints]
    msg.hard_path_link_constraints = [
        convert_to_tsr_link_constraint_msg(x) for x in goal_rosbag.constraints.hard_path_link_constraints]
    pub.publish(msg)


def convert_generated_trajectories(trajectories_rosbag: DisplayTrajectoryRosBag) -> DisplayTrajectory:
    msg = DisplayTrajectory()
    for trajectory_rosbag in trajectories_rosbag.trajectory:
        trajectory_msg = RobotTrajectory()
        trajectory_msg.joint_trajectory.joint_names = trajectory_rosbag.joint_trajectory.joint_names
        for point_rosbag in trajectory_rosbag.joint_trajectory.points:
            point_msg = JointTrajectoryPoint()
            point_msg.positions = point_rosbag.positions.tolist()
            point_msg.time_from_start.sec = point_rosbag.time_from_start.sec
            point_msg.time_from_start.nanosec = point_rosbag.time_from_start.nanosec
            trajectory_msg.joint_trajectory.points.append(point_msg)

        trajectory_msg.multi_dof_joint_trajectory.joint_names =\
            trajectory_rosbag.multi_dof_joint_trajectory.joint_names
        for point_rosbag in trajectory_rosbag.multi_dof_joint_trajectory.points:
            point_msg = MultiDOFJointTrajectoryPoint()
            transform = Transform()
            transform.translation.x = point_rosbag.transforms[0].translation.x
            transform.translation.y = point_rosbag.transforms[0].translation.y
            transform.translation.z = point_rosbag.transforms[0].translation.z
            transform.rotation.x = point_rosbag.transforms[0].rotation.x
            transform.rotation.y = point_rosbag.transforms[0].rotation.y
            transform.rotation.z = point_rosbag.transforms[0].rotation.z
            transform.rotation.w = point_rosbag.transforms[0].rotation.w
            point_msg.transforms.append(transform)
            point_msg.time_from_start.sec = point_rosbag.time_from_start.sec
            point_msg.time_from_start.nanosec = point_rosbag.time_from_start.nanosec
            trajectory_msg.multi_dof_joint_trajectory.points.append(point_msg)

        msg.trajectory.append(trajectory_msg)
    return msg


def publish_generated_trajectories(pub: rclpy.publisher.Publisher, trajectories: DisplayTrajectory) -> None:
    if trajectories is None:
        return
    pub.publish(trajectories)


class TopicStorage:

    def __init__(self, reader: Reader, typestore: rosbags.typesys.store.Typestore, topic_name: str,
                 convert_func: Callable[[Any], Any] = None) -> None:
        self._topics = []
        connections = [x for x in reader.connections if x.topic == topic_name]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            if convert_func is None:
                self._topics.append((timestamp, msg))
            else:
                self._topics.append((timestamp, convert_func(msg)))
        if not self._topics:
            raise
        self._topics.sort()
        self._value = None

    def extract(self, stamp_nsec: int, change_only: bool = False) -> Any:
        if not self._topics:
            if change_only:
                return None
            else:
                return self._value

        new_value_found = False
        while self._topics and self._topics[0][0] < stamp_nsec:
            self._value = self._topics.pop(0)[1]
            new_value_found = True
        if change_only:
            if new_value_found:
                return self._value
            else:
                return None
        else:
            return self._value


def main():
    rclpy.init()
    node = rclpy.create_node('rosbag_player')

    broadcaster = tf2_ros.StaticTransformBroadcaster(node)
    joint_states_pub = node.create_publisher(JointState, 'debug_joint_state', 1)
    environment_marker_pub = node.create_publisher(MarkerArray, 'collision_environment_server/marker_array', 1)
    attached_marker_pub = node.create_publisher(MarkerArray, 'attached_object_publisher/marker_array', 1)
    constraints_pub = node.create_publisher(Constraints, 'link_constraints_visualization/constraints', 1)
    generated_trajectories_pub = node.create_publisher(DisplayTrajectory, 'generated_trajectories', 1)

    parser = argparse.ArgumentParser()
    parser.add_argument('rosbag2_dir')
    parser.add_argument('--start-offset', type=float, default=0.0)
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--tmc-planning-msgs-share-dir',
                        default=os.environ['HOME'] + '/hsre_ws/install/tmc_planning_msgs/share')
    parser.add_argument('--publish-generated-trajectories', action='store_true')
    parser.add_argument('--start-paused', action='store_true')
    args = parser.parse_args()

    if args.rate < 0.0:
        raise

    add_types = {}
    for name in ['RobotLocalGoal', 'Constraints', 'TsrLinkConstraint', 'TaskSpaceRegion', 'RangeJointConstraint']:
        add_types.update(get_types_from_msg(
            Path(args.tmc_planning_msgs_share_dir + '/tmc_planning_msgs/msg/' + name + '.msg').read_text(),
            'tmc_planning_msgs/msg/' + name))
    for name in ['RobotState', 'AttachedCollisionObject', 'CollisionObject', 'DisplayTrajectory', 'RobotTrajectory']:
        add_types.update(get_types_from_msg(
            Path('/opt/ros/humble/share/moveit_msgs/msg/' + name + '.msg').read_text(),
            'moveit_msgs/msg/' + name))
    add_types.update(get_types_from_msg(
        Path('/opt/ros/humble/share/control_msgs/msg/JointTrajectoryControllerState.msg').read_text(),
        'control_msgs/msg/JointTrajectoryControllerState'))
    add_types.update(get_types_from_msg(
        Path('/opt/ros/humble/share/object_recognition_msgs/msg/ObjectType.msg').read_text(),
        'object_recognition_msgs/msg/ObjectType'))

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    typestore.register(add_types)

    with Reader(args.rosbag2_dir) as reader:
        start_stamp_nsec = reader.start_time
        end_stamp_nsec = reader.end_time

        joint_states_topics = TopicStorage(reader, typestore, '/joint_states')
        base_states_topics = TopicStorage(reader, typestore, '/omni_base_controller/state')
        environment_markers_topics = TopicStorage(reader, typestore, '/collision_environment_server/marker_array')
        attached_markers_topics = TopicStorage(reader, typestore, '/attached_object_publisher/marker_array')
        robot_local_goals_topics = TopicStorage(reader, typestore, '/hsrb_robot_local_planner/constraints')
        if args.publish_generated_trajectories:
            generated_trajectories_topics = TopicStorage(
                reader, typestore, '/hsrb_robot_local_planner/generated_trajectories', convert_generated_trajectories)

    # I won't do my best to manage the time
    time_from_start_nsec = int(args.start_offset * 1.0e9)
    is_running = not args.start_paused
    node.get_logger().info("Loaded rosbag")
    if not is_running:
        node.get_logger().info('Start with r.')
    while start_stamp_nsec + time_from_start_nsec < end_stamp_nsec:
        time.sleep(0.02)
        key = get_key()
        if key == 115:  # s
            is_running = False
            node.get_logger().info('Stopped. Restart with r.')
        elif key == 114:  # r
            is_running = True
            node.get_logger().info('Restarted.')
        elif key == 113:  # q
            return

        if is_running:
            time_from_start_nsec += int(2e7 * args.rate)

        current_stamp = start_stamp_nsec + time_from_start_nsec
        publish_joint_state(joint_states_pub, joint_states_topics.extract(current_stamp))
        broadcast_base_state(broadcaster, node, base_states_topics.extract(current_stamp))
        publish_marker_array(environment_marker_pub, environment_markers_topics.extract(current_stamp))
        publish_marker_array(attached_marker_pub, attached_markers_topics.extract(current_stamp))
        publish_constraints_from_goal(constraints_pub, robot_local_goals_topics.extract(current_stamp, True))
        if args.publish_generated_trajectories:
            publish_generated_trajectories(generated_trajectories_pub,
                                           generated_trajectories_topics.extract(current_stamp, True))
            if key == 103 and not is_running:  # g
                publish_generated_trajectories(generated_trajectories_pub,
                                               generated_trajectories_topics.extract(current_stamp))


if __name__ == '__main__':
    main()
