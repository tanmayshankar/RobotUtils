#!/usr/bin/env python2

import rospy, copy
from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from std_srvs.srv import Empty
from baxter_pykdl import baxter_kinematics
import threading

class RobotResetManager():

	def __init__(self, movegroup_interface):

		# Initialize objects to call services to pause and unpause Gazebo physics. 
		self.pause_physics = rospy.ServiceProxy("/gazebo/pause_physics",Empty)
		self.unpause_physics = rospy.ServiceProxy("/gazebo/unpause_physics",Empty)

		# Create objects to call services to set model configuration. 
		self.set_config_service = rospy.ServiceProxy("/gazebo/set_model_configuration",SetModelConfiguration)

		# Create object to call services to start and stop controllers. 
		self.switch_controller_service = rospy.ServiceProxy("/robot/controller_manager/switch_controller",SwitchController)

		# Create service requests for the set config and switch controller services. 
		self.config_request = SetModelConfigurationRequest()
		self.config_request.model_name = "baxter"
		self.config_request.urdf_param_name = "robot_description"
		# Set default joint names and joint positions. 
		# Can also set things for left arm by just providing corresponding dictionary. 
		self.config_request.joint_names = ["right_s0","right_s1","right_e0","right_e1","right_w0","right_w1","right_w2"]
		self.config_request.joint_positions = [0.,0.,0.,0.,0.,0.,0.]

		# Creating service request to switch controllers on and off separately. 
		# Request to switch on controllers. 
		self.switch_controllers_on_request = SwitchControllerRequest()
		# self.switch_controllers_on_request.start_controllers = ['joint_state_controller','left_joint_position_controller','right_joint_position_controller']
		self.switch_controllers_on_request.start_controllers = ['joint_state_controller']
		self.switch_controllers_on_request.stop_controllers = []
		self.switch_controllers_on_request.strictness = 2
		
		# Request to switch off controllers. 
		self.switch_controllers_off_request = SwitchControllerRequest()
		self.switch_controllers_off_request.start_controllers = []
		# self.switch_controllers_off_request.stop_controllers = ['joint_state_controller','left_joint_position_controller','right_joint_position_controller']
		self.switch_controllers_off_request.stop_controllers = ['joint_state_controller']
		self.switch_controllers_off_request.strictness = 2

		# Simulation reset. 
		self.reset_sim_service = rospy.ServiceProxy("/gazebo/reset_simulation",Empty)

		# Baxter PyKDL objects. 
		self.baxter_right_kin_obj = baxter_kinematics('right')
		self.baxter_left_kin_obj = baxter_kinematics('left')

		self.movegroup = movegroup_interface

	def set_to_joint_pose(self, joint_positions, joint_names=None): 
		# (0) Reset and Enable. 
		self.movegroup.reset_and_enable()

		# (1) Disable Controllers. 
		self.switch_controller_service(self.switch_controllers_off_request)
		
		# (2) Pause Physics.
		self.pause_physics()
		
		# (3) Set Model Configuration. 
		if joint_names:
			self.config_request.joint_names = joint_names		
		self.config_request.joint_positions = joint_positions
		self.set_config_service(self.config_request)
		self.set_config_service(self.config_request)

		# (4) Reset Controllers.
		# self.switch_controller_service(self.switch_controllers_off_request)
		controller_thread = threading.Thread(target=self.switch_controller_service.call,args=[self.switch_controllers_on_request])
		controller_thread.start()

		# (5) Unpause Phyiscs, and Immediately exit control mode. 
		self.unpause_physics()
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

	def hard_reset(self):

		# If we do something like cross joint limits of the baxter and need a hard reset of the simulation entirely. 
		self.reset_sim_service()
		self.movegroup.reset_and_enable()
		
	def check_and_reset(self):

		# Check if we exceeded joint angles, and then reset if so. 
		current_joint_angles = self.movegroup.right_limb.joint_angles()

		if not((self.baxter_right_kin_obj.joint_limits_lower < current_joint_angles.values()).all() and (current_joint_angles.values() < self.baxter_right_kin_obj.joint_limits_upper).all()):
			print("Hard reset!")
			self.hard_reset()	
