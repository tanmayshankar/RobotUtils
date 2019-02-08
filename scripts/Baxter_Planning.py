#!/usr/bin/env python2

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt! interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:
##

import sys, copy, rospy, moveit_commander, numpy as np
import moveit_msgs.msg, geometry_msgs.msg
from math import pi
import torch
from IPython import embed
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
	"""
	Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
	@param: goal       A list of floats, a Pose or a PoseStamped
	@param: actual     A list of floats, a Pose or a PoseStamped
	@param: tolerance  A float
	@returns: bool
	"""
	all_equal = True
	if type(goal) is list:
		for index in range(len(goal)):
			if abs(actual[index] - goal[index]) > tolerance:
				return False

	elif type(goal) is geometry_msgs.msg.PoseStamped:
		return all_close(goal.pose, actual.pose, tolerance)

	elif type(goal) is geometry_msgs.msg.Pose:
		return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

	return True

class MoveGroupPythonInterface(object):
	"""MoveGroupPythonIntefaceTutorial"""
	def __init__(self):
		super(MoveGroupPythonInterface, self).__init__()

		## First initialize `moveit_commander`_ and a `rospy`_ node:
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

		# Instantiate a `RobotCommander`_ object. This object is the outer-level interface to the robot:
		robot = moveit_commander.RobotCommander()

		# Instantiate a `PlanningSceneInterface`_ object.  This object is an interface to the world surrounding the robot:
		scene = moveit_commander.PlanningSceneInterface()

		# Instantiate a `MoveGroupCommander`_ object.  This object is an interface to one group of joints. 
		group_name = "left_arm"
		group = moveit_commander.MoveGroupCommander(group_name)

		# We create a `DisplayTrajectory`_ publisher which is used later to publish trajectories for RViz to visualize:
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)	

		# ## Getting Basic Information
		# We can get the name of the reference frame for this robot:
		planning_frame = group.get_planning_frame()

		# We can also print the name of the end-effector link for this group:
		eef_link = group.get_end_effector_link()

		# We can get a list of all the groups in the robot: 
		group_names = robot.get_group_names()	

		# Sometimes for debugging it is useful to print the entire state of the robot:	
		# print robot.get_current_state()

		# Misc variables
		self.box_name = ''
		self.robot = robot
		self.scene = scene
		self.group = group
		self.display_trajectory_publisher = display_trajectory_publisher
		self.planning_frame = planning_frame
		self.eef_link = eef_link
		self.group_names = group_names

	def go_to_joint_state(self, joint_goal):	
		# Planning to a Joint Goal

		# joint_goal = self.group.get_current_joint_values()

		# The go command can be called with joint values, poses, or without any
		# parameters if you have already set the pose or joint target for the group
		# plan = self.group.go(joint_goal, wait=True)

		plan = self.group.plan(joint_goal)
		self.group.execute(plan, wait=True)		
		self.group.stop()	

		current_joints = self.group.get_current_joint_values()
		return all_close(joint_goal, current_joints, 0.01), plan

	def go_to_pose_goal(self, pose_goal=None):
		# Planning to a Pose Goal
		
		# We can plan a motion for this group to a desired pose for the end-effector:
		if pose_goal==None:
			pose_goal = geometry_msgs.msg.Pose()		
			pose_goal.orientation.w = 1.0
			pose_goal.position.x = 0.4
			pose_goal.position.y = -0.1
			pose_goal.position.z = 0.4
		self.group.set_pose_target(pose_goal)

		# Here, we can run plan without specifying a  joint angle target, because we have already set a pose target above. 
		plan = self.group.plan()
		self.group.execute(plan, wait=True)
		self.group.stop()
	
		# It is always good to clear your targets after planning with poses.
		# Note: there is no equivalent function for clear_joint_value_targets()
		self.group.clear_pose_targets()

		current_pose = self.group.get_current_pose().pose
		return all_close(pose_goal, current_pose, 0.01), plan

	def plan_cartesian_path(self, scale=1):
		## BEGIN_SUB_TUTORIAL plan_cartesian_path
		##
		## Cartesian Paths
		## ^^^^^^^^^^^^^^^
		## You can plan a Cartesian path directly by specifying a list of waypoints
		## for the end-effector to go through:
		##
		waypoints = []

		wpose = self.group.get_current_pose().pose
		wpose.position.z -= scale * 0.1  # First move up (z)
		wpose.position.y += scale * 0.2  # and sideways (y)
		waypoints.append(copy.deepcopy(wpose))

		wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
		waypoints.append(copy.deepcopy(wpose))

		wpose.position.y -= scale * 0.1  # Third move sideways (y)
		waypoints.append(copy.deepcopy(wpose))

		# We want the Cartesian path to be interpolated at a resolution of 1 cm
		# which is why we will specify 0.01 as the eef_step in Cartesian
		# translation.  We will disable the jump threshold by setting it to 0.0 disabling:
		(plan, fraction) = self.group.compute_cartesian_path(waypoints,   # waypoints to follow
															 0.01,        # eef_step
															 0.0)         # jump_threshold

		# Note: We are just planning, not asking move_group to actually move the robot yet:
		return plan, fraction

		## END_SUB_TUTORIAL

	def display_trajectory(self, plan):
		display_trajectory_publisher = self.display_trajectory_publisher

		## Displaying a Trajectory
		## ^^^^^^^^^^^^^^^^^^^^^^^
		## You can ask RViz to visualize a plan (aka trajectory) for you. But the
		## group.plan() method does this automatically so this is not that useful
		## here (it just displays the same trajectory again):
		##
		## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
		## We populate the trajectory_start with our current robot state to copy over
		## any AttachedCollisionObjects and add our plan to the trajectory.
		display_trajectory = moveit_msgs.msg.DisplayTrajectory()
		display_trajectory.trajectory_start = self.robot.get_current_state()
		display_trajectory.trajectory.append(plan)
		# Publish
		display_trajectory_publisher.publish(display_trajectory);

	def parse_plan(self, plan, dofs=7):

		traj_length = len(plan.joint_trajectory.points)
		plan_array = np.zeros((traj_length, dofs))
		# Parses plan into numpy array of len (T x DoF)
		for t in range(traj_length):
			plan_array[t] = plan.joint_trajectory.points[t].positions

		return plan_array			

def main():
	try:
		movegroup = MoveGroupPythonInterface()

		# joint_goal = movegroup.group.get_current_joint_values()
		# joint_goal[0] += 0.2
		# joint_goal[2] += 0.2
		# joint_goal[6] += 0.2
		# success, plan = movegroup.go_to_joint_state(joint_goal)	
		
		success, plan = movegroup.go_to_pose_goal()
		embed()
		plan_array = movegroup.parse_plan(plan)
		embed()

	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
	main()
