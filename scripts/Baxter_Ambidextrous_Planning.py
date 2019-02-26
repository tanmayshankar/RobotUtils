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

import sys, copy, rospy, moveit_commander, roslib, time
import moveit_msgs.msg, geometry_msgs.msg, cv2
from math import pi
import torch, numpy as np
from IPython import embed
from std_msgs.msg import String, Header
from moveit_commander.conversions import pose_to_list
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState, Image
from moveit_msgs.msg import RobotState
from cv_bridge import CvBridge, CvBridgeError
from baxter_core_msgs.srv import (
			SolvePositionIK,
				SolvePositionIKRequest,
				)
import baxter_interface
import matplotlib.pyplot as plt

class ImageRetriever():

	def __init__(self):
		# rospy.init_node('image_converter', anonymous=True)
		self.bridge = CvBridge()
		self.image1_sub = rospy.Subscriber("/rviz1/camera1/image",Image,self.callback1)
		self.image2_sub = rospy.Subscriber("/rviz1/camera2/image",Image,self.callback2)
		self.image3_sub = rospy.Subscriber("/rviz1/camera3/image",Image,self.callback3)		

	def callback1(self, data):
		try:	    		
		  self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
		  print(e)		

	def callback2(self, data):
		try:
		  self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
		  print(e)		

	def callback3(self, data):
		try:
		  self.cv_image3 = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
		  print(e)		

	def retrieve_image(self, camera_id):
		if camera_id==1:
			return self.cv_image1
		if camera_id==2:
			return self.cv_image2
		if camera_id==3:
			return self.cv_image3

class MoveGroupPythonInterface(object):
	"""MoveGroupPythonIntefaceTutorial"""
	def __init__(self):
		super(MoveGroupPythonInterface, self).__init__()

		## First initialize `moveit_commander`_ and a `rospy`_ node:
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('move_group_python_interface', anonymous=True)

		# Instantiate a `RobotCommander`_ object. This object is the outer-level interface to the robot:
		self.robot = moveit_commander.RobotCommander()

		# Instantiate a `PlanningSceneInterface`_ object.  This object is an interface to the world surrounding the robot:
		self.scene = moveit_commander.PlanningSceneInterface()

		# Instantiate a `MoveGroupCommander`_ object.  This object is an interface to one group of joints. 		
		self.left_arm = moveit_commander.MoveGroupCommander("left_arm")
		self.right_arm = moveit_commander.MoveGroupCommander("right_arm")		

		self.left_limb = baxter_interface.Limb('left')
		self.right_limb = baxter_interface.Limb('right')
		self.left_gripper = baxter_interface.Gripper('left', baxter_interface.CHECK_VERSION)
		self.right_gripper = baxter_interface.Gripper('right', baxter_interface.CHECK_VERSION)

		# Get end effector links for FK. 
		self.left_fk = [self.left_arm.get_end_effector_link()]
		self.right_fk = [self.right_arm.get_end_effector_link()]

		self.header = Header(0,rospy.Time.now(),"base")
		self.joints_info = RobotState()

		self.joint_names = ['head_nod', 'head_pan', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2',
		 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'torso_t0']

		rospy.wait_for_service('compute_fk')

		try:
			self.moveit_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
			# self.IK_namespace = 'compute_ik'
			limb="right"
			self.IK_namespace = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
			self.moveit_IK = rospy.ServiceProxy(self.IK_namespace, SolvePositionIK)
			self.IK_request = SolvePositionIKRequest()			
		except rospy.ServiceException as e:
			rospy.logerror("Service call failed: %s"%e)

		# We can get a list of all the groups in the robot: 
		self.group_names = self.robot.get_group_names()	

		# Sometimes for debugging it is useful to print the entire state of the robot:	
		# print robot.get_current_state()

	def all_close(self, goal, actual, tolerance):
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
			return self.all_close(goal.pose, actual.pose, tolerance)

		elif type(goal) is geometry_msgs.msg.Pose:
			return self.all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

		return True

	def plan_to_joint_state(self, arm, joint_goal):
		# Planning to a Joint Goal

		if arm=='left':
			group = self.left_arm
			offset = 2
		elif arm=='right':
			group = self.right_arm
			offset = 9

		# joint_goal = self.group.get_current_joint_values()

		# The go command can be called with joint values, poses, or without any
		# parameters if you have already set the pose or joint target for the group
		# plan = self.group.go(joint_goal, wait=True)

		# Construct RobotState object for the planner. 
		joints_info = RobotState()

		# CAN TAKE IN SUBSET OF JOINT ANGLES.
		joints_info.joint_state.name = self.joint_names[offset:offset+7]
		joints_info.joint_state.position = joint_goal

		plan = None 
		try:
			plan = group.plan(joints_info)
		except: 
			print("Plan failed.")		

		return plan

	def go_to_joint_state(self, arm, joint_goal):

		plan = self.plan_to_joint_state(arm, joint_goal)
		if plan:
			group.execute(plan, wait=True)		
			group.stop()	
			current_joints = group.get_current_joint_values()
			self.all_close(joint_goal, current_joints, 0.01)

		return plan

	def go_to_pose_goal(self, arm, pose_goal=None):
		# Planning to a Pose Goal
		
		if arm=='left':
			group = self.left_arm
		elif arm=='right':
			group = self.right_arm

		# We can plan a motion for this group to a desired pose for the end-effector:
		if pose_goal==None:
			pose_goal = geometry_msgs.msg.Pose()
			pose_goal.orientation.w = 1.0
			pose_goal.position.x = 0.2
			pose_goal.position.y = 0.1
			pose_goal.position.z = 0.4
		group.set_pose_target(pose_goal)

		# Here, we can run plan without specifying a  joint angle target, because we have already set a pose target above. 
		plan = group.plan()
		group.execute(plan, wait=True)
		group.stop()
	
		# It is always good to clear your targets after planning with poses.
		# Note: there is no equivalent function for clear_joint_value_targets()
		group.clear_pose_targets()

		current_pose = group.get_current_pose().pose
		self.all_close(pose_goal, current_pose, 0.01)
		return plan

	def plan_cartesian_path(self, arm, scale=1):
		## BEGIN_SUB_TUTORIAL plan_cartesian_path
		##
		## Cartesian Paths
		## ^^^^^^^^^^^^^^^
		## You can plan a Cartesian path directly by specifying a list of waypoints
		## for the end-effector to go through:
		##

		if arm=='left':
			group = self.left_arm
		elif arm=='right':
			group = self.right_arm

		waypoints = []

		wpose = group.get_current_pose().pose
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
		(plan, fraction) = group.compute_cartesian_path(waypoints,   # waypoints to follow
															 0.01,        # eef_step
															 0.0)         # jump_threshold

		# Note: We are just planning, not asking move_group to actually move the robot yet:
		return plan, fraction

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

	def select_arm_joint_angle(self, arm, joint_angle_trajectory):
		if arm=="right":
			# Use angles indexed 9 to 16 (included.)
			offset = 9
		elif arm=="left":
			# Use angles indexed 2 to 8 (included.)
			offset = 2
		return joint_angle_trajectory[:,offset:offset+7]		

	def recreate_dictionary(self, arm, joint_angles):
		if arm=="left":
			offset = 2           
		elif arm=="right":
			offset = 9
		return dict((self.joint_names[i],joint_angles[i-offset]) for i in range(offset,offset+7))

	def Compute_FK(self, arm, joint_dict):
		# Remember, moveit_fk takes in a RobotState object. 
		self.joints_info = RobotState()

		# CAN TAKE IN SUBSET OF JOINT ANGLES.
		self.joints_info.joint_state.name = joint_dict.keys()
		self.joints_info.joint_state.position = joint_dict.values()

		if arm=='left':
			fk_instance = self.left_fk			
		elif arm=='right':
			fk_instance = self.right_fk
		
		# RETURNS STAMPED POSE.
		pose = self.moveit_fk(self.header, fk_instance, self.joints_info)
		return pose

	def Compute_IK(self, arm, end_effector_pose):

		# Parse pose into pose object. 
		pose_obj = self.parse_into_pose(end_effector_pose)

		# First push the pose down the list of poses for which we request IK. 
		self.IK_request.pose_stamp.append(pose_obj)

		try: 
			# Wait 5 seconds in life for the IK service to be called with the IK request message. 
			rospy.wait_for_service(self.IK_namespace, 5.0)
			# Get the result by calling the IK service on the IK request message. 
			resp = self.moveit_IK(self.IK_request)

		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
			return 1

		# Adapted from IK_Service_Client
		# Check if result valid, and type of seed ultimately used to get solution
		# convert rospy's string representation of uint8[]'s to int's
		resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
		if (resp_seeds[0] != resp.RESULT_INVALID):
			seed_str = { ikreq.SEED_USER: 'User Provided Seed',
						ikreq.SEED_CURRENT: 'Current Joint Angles',
						ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
					   }.get(resp_seeds[0], 'None')
			print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" % (seed_str,))
			# Format solution into Limb API-compatible dictionary
			limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
			print "\nIK Joint Solution:\n", limb_joints
			print "------------------"
			print "Response Message:\n", resp
		else:
			limb_joints = None
			print("INVALID POSE - No Valid Joint Solution Found.")

		return limb_joints


	def parse_fk_plan(self, arm, plan, dofs=7):
		traj_length = len(plan.joint_trajectory.points)

		# Create array of size T x DoF to store end effector trajectory. 
		# EE Traj stored as X,Y,Z,Qx,Qy,Qz,Qw.
		plan_array = np.zeros((traj_length, dofs))
		# For every timepoint in the trajectory, 
		for t in range(traj_length):
			# Retrieve joint angles from plan. 
			joint_angles = plan.joint_trajectory.points[t].positions
			# Recreate dict for FK. 
			joint_angle_dict = self.recreate_dictionary(arm, joint_angles)
			# Compute FK.
			end_effector_pose = self.Compute_FK(arm, joint_angle_dict)
			# Parse pose into array. 
			plan_array[t] = self.parse_pose(end_effector_pose)

		return plan_array	

	def parse_pose(self, pose_object):

		pose_array = np.zeros((7))
		pose_array[0] = pose_object.pose_stamped[0].pose.position.x
		pose_array[1] = pose_object.pose_stamped[0].pose.position.y
		pose_array[2] = pose_object.pose_stamped[0].pose.position.z
		pose_array[3] = pose_object.pose_stamped[0].pose.orientation.x
		pose_array[4] = pose_object.pose_stamped[0].pose.orientation.y
		pose_array[5] = pose_object.pose_stamped[0].pose.orientation.z
		pose_array[6] = pose_object.pose_stamped[0].pose.orientation.w
		return pose_array

	def parse_into_pose(self, state):

		pose_object = geometry_msgs.msg.PoseStamped()
		pose_object.header = self.header
		pose_object.pose.position.x = state[0]
		pose_object.pose.position.y = state[1]
		pose_object.pose.position.z = state[2]
		pose_object.pose.orientation.x = state[3]
		pose_object.pose.orientation.y = state[4]
		pose_object.pose.orientation.z = state[5]
		pose_object.pose.orientation.w = state[6]

		return pose_object

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
		image_retriever = ImageRetriever()		
		time.sleep(5)

		# movegroup.right_limb.move_to_neutral()		
		# movegroup.left_limb.move_to_neutral()		
		joint_goal = movegroup.right_arm.get_current_joint_values()
		# joint_goal = movegroup.left_arm.get_current_joint_values()
		joint_goal[2] += 0.2

		plan = movegroup.go_to_joint_state('left',joint_goal)	
		plan_array = movegroup.parse_fk_plan('left',plan)
		embed()

	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
	main()
