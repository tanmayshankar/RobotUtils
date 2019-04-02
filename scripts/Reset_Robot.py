#!/usr/bin/env python2

import rospy, copy
from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from std_srvs.srv import Empty
from baxter_pykdl import baxter_kinematics
import threading, numpy as np
from IPython import embed

class RobotResetManager():

	def __init__(self, movegroup_interface):

		# Initialize objects to call services to pause and unpause Gazebo physics. 
		self.pause_physics = rospy.ServiceProxy("/gazebo/pause_physics",Empty)
		self.unpause_physics = rospy.ServiceProxy("/gazebo/unpause_physics",Empty)

		# Create objects to call services to set model configuration. 
		self.set_config_service = rospy.ServiceProxy("/gazebo/set_model_configuration",SetModelConfiguration)

		# Create object to call services to start and stop controllers. 
		self.switch_controller_service = rospy.ServiceProxy("/robot/controller_manager/switch_controller",SwitchController)

		# Simulation reset. 
		self.reset_world_service = rospy.ServiceProxy("/gazebo/reset_world",Empty)
		self.reset_sim_service = rospy.ServiceProxy("/gazebo/reset_simulation",Empty)

		# Baxter PyKDL objects. 
		self.baxter_right_kin_obj = baxter_kinematics('right')
		self.baxter_left_kin_obj = baxter_kinematics('left')
		# Order of joint names that joint limits are stored as. 
		# jn =  ["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"]

		self.movegroup = movegroup_interface
		# Create dictionaries of joint limits.
		self.joint_limit_lower_dict = self.movegroup.recreate_dictionary("right", self.baxter_right_kin_obj.joint_limits_lower, self.baxter_right_kin_obj._joint_names)
		self.joint_limit_upper_dict = self.movegroup.recreate_dictionary("right", self.baxter_right_kin_obj.joint_limits_upper, self.baxter_right_kin_obj._joint_names)
		# Create sorted values. 
		self.lower_limits = np.array([value for (key, value) in sorted(self.joint_limit_lower_dict.items())]) - 0.0005*np.ones((7))
		self.upper_limits = np.array([value for (key, value) in sorted(self.joint_limit_upper_dict.items())]) + 0.0005*np.ones((7))

	def setup(self):
		self.set_config_request()
		self.set_controller_on_request()
		self.set_controller_off_request()

	def set_config_request(self, joint_names=None, joint_positions=None):
		# Create service requests for the set config and switch controller services. 
		self.config_request = SetModelConfigurationRequest()
		self.config_request.model_name = "baxter"
		self.config_request.urdf_param_name = "robot_description"
		# Set default joint names and joint positions. 
		# Can also set things for left arm by just providing corresponding dictionary. 
		if (joint_names is None) and (joint_positions is None):
			self.config_request.joint_names = ["right_s0","right_s1","right_e0","right_e1","right_w0","right_w1","right_w2"]
			self.config_request.joint_positions = [0.,0.,0.,0.,0.,0.,0.]
		if joint_names is not None:
			self.config_request.joint_names = joint_names
		if joint_positions is not None:
			self.config_request.joint_positions = joint_positions

	def set_controller_on_request(self):
		# Creating service request to switch controllers on and off separately. 
		# Request to switch on controllers. 
		self.switch_controllers_on_request = SwitchControllerRequest()
		# self.switch_controllers_on_request.start_controllers = ['joint_state_controller','left_joint_position_controller','right_joint_position_controller']
		self.switch_controllers_on_request.start_controllers = ['joint_state_controller']
		self.switch_controllers_on_request.stop_controllers = []
		self.switch_controllers_on_request.strictness = 2
		
	def set_controller_off_request(self):
		# Request to switch off controllers. 
		self.switch_controllers_off_request = SwitchControllerRequest()
		self.switch_controllers_off_request.start_controllers = []
		# self.switch_controllers_off_request.stop_controllers = ['joint_state_controller','left_joint_position_controller','right_joint_position_controller']
		self.switch_controllers_off_request.stop_controllers = ['joint_state_controller']
		self.switch_controllers_off_request.strictness = 2

	def set_to_joint_pose(self, joint_positions, joint_names=None): 
		# (0) Reset and Enable. 
		self.movegroup.reset_and_enable()

		# (1) Disable Controllers. 
		self.switch_controller_service(self.switch_controllers_off_request)
		
		# (2) Pause Physics.
		self.pause_physics()
		
		# (3) Set Model Configuration. 
		self.set_config_request(joint_names, joint_positions)
		self.set_config_service(self.config_request)
		self.set_config_service(self.config_request)

		# (4) Reset Controllers.
		# self.switch_controller_service(self.switch_controllers_off_request)
		controller_thread = threading.Thread(target=self.switch_controller_service.call,args=[self.switch_controllers_on_request])
		controller_thread.start()

		# (5) Unpause Phyiscs, and Immediately exit control mode. 
		self.unpause_physics()
		self.movegroup.right_limb.exit_control_mode()
		self.movegroup.right_limb.exit_control_mode()
		self.movegroup.left_limb.exit_control_mode()

	def set_to_end_effector_pose(self, end_effector_pose, arm="right",ik="default"):
		# Perform IK.
		# Call set_to_joint_pose on the result. 
		if ik=="default":
			joint_dict = self.movegroup.Alt_Compute_IK(arm,end_effector_pose)
		else:
			if arm=="right":
				joint_positions = self.baxter_right_kin_obj.inverse_kinematics(end_effector_pose[:3],orientation=end_effector_pose[3:])
			elif arm=="left":
				joint_positions = self.baxter_left_kin_obj.inverse_kinematics(end_effector_pose[:3],orientation=end_effector_pose[3:])
			
			if joint_positions is not None:
				joint_dict = self.movegroup.recreate_dictionary(arm, joint_positions)

		
		if joint_dict is not None:
			self.set_to_joint_pose(joint_dict.values(), joint_names=joint_dict.keys())
			return True
		else:
			return False

	def hard_reset(self):

		# If we do something like cross joint limits of the baxter and need a hard reset of the simulation entirely. 
		# self.reset_sim_service()	
		self.reset_world_service()
		self.movegroup.right_limb.move_to_neutral()
		self.movegroup.reset_and_enable()

	def check_bounds(self):
		# Check if we exceeded joint angles, and then reset if so. 
		current_joint_angles = self.movegroup.right_limb.joint_angles()

		# Sort the values. 
		joint_angle_values = np.array([value for (key, value) in sorted(current_joint_angles.items())])

		if not((self.lower_limits < joint_angle_values).all() and (joint_angle_values < self.upper_limits).all()):
			print("Out of bounds!")
			return False		
		return True
		
	def check_and_reset(self):
		if not(self.check_bounds()):
			self.hard_reset()
			return False
		return True