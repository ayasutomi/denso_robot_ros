#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
from sensor_msgs.msg import JointState
import sys
from copy import copy


class RobotArm(object):
    def __init__(self, robot_name):

        self.robot_name = robot_name
        self._client = actionlib.SimpleActionClient(robot_name + \
            "/arm_controller/follow_joint_trajectory",FollowJointTrajectoryAction)

        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    

    def get_current_joint_pos(self):
        joint_states = rospy.wait_for_message(self.robot_name + "/joint_states", JointState, 5)
        return list(joint_states.position)
    
class RobotHand(object):
    def __init__(self, robot_name):

        self.robot_name = robot_name
        self._client = actionlib.SimpleActionClient(robot_name + \
            "/hand_controller/follow_joint_trajectory",FollowJointTrajectoryAction)

        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [ 'panda_finger_joint1', 'panda_finger_joint2']
    

    def get_current_joint_pos(self):
        joint_states = rospy.wait_for_message(self.robot_name + "/joint_states", JointState, 5)
        return list(joint_states.position)

    def openHand(self):
        self.clear()
        current = self.get_current_joint_pos()
        self.add_point(current[6:8],0)
        self.add_point([0.04 , 0.04], 1)
        self.start()
        self.wait()

    def closeHand(self):
        self.clear()
        current = self.get_current_joint_pos()
        self.add_point(current[6:8],0)
        self.add_point([0.0 , 0.0], 1)
        self.start()
        self.wait()

        





