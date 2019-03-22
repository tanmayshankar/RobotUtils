#!/usr/bin/env python2
from Baxter_Ambidextrous_Planning import *
from Reset_Robot import RobotResetManager
from baxter_pykdl import baxter_kinematics
from moveit_msgs.srv import GetStateValidity

def main():	
	try:

		movegroup = MoveGroupPythonInterface()
		print("Created Movegroup.")
		# image_retriever = ImageRetriever()		
		# print("Created Image Retriever.")
		reset_manager = RobotResetManager(movegroup)
		print("Created Robot Manager.")
		bax_kinematics = baxter_kinematics('right')
		print("Created Kinematics Object.")

		joint_states = np.load("Joint_States.npy")
		end_effector_states = np.load("EE_States.npy")
		joint_state_seeds = np.load("Joint_State_Seeds.npy")		
		joint_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']

		N = 1000
		K = 5
		dofs = 7

		solution_matrix = np.zeros((2, N, K))
		resultant_joint_states = np.zeros((2, N, dofs, K))

		seed_state = JointState()
		seed_state.name = joint_names

		for i in range(N):
			for j in range(K):

				print("Running:",i,j)

				# ################# # 
				# For both the baxter_pykdl and original IK service, feed in the EE state, and the joint seed, collect resultant state. 
				
				# FOR ORIGINAL IK SERVICE: 				
				# Set pose in request. 
				pose_obj = movegroup.parse_into_pose(end_effector_states[i])
				movegroup.IK_request.pose_stamp = [pose_obj]

				# Set seed. 
				seed_state.position = joint_states[i]+joint_state_seeds[i,:,j]				
				movegroup.IK_request.seed_angles = [seed_state]

				# Call IK. 
				IK_result = movegroup.inverse_kinematics_service(movegroup.IK_request)

				if IK_result.isValid[0]:
					solution_matrix[0,i,j] = 1.
					resultant_joint_states[0,i,:,j] = IK_result.joints[0].position

				# ################# # 

				# For Baxter_PyKDL: 
				result = bax_kinematics.inverse_kinematics(end_effector_states[i,:3],orientation=end_effector_states[i,3:],seed=joint_states[i]+joint_state_seeds[i,:,j])

				if result is not None:
					solution_matrix[1,i,j] = 1.
					resultant_joint_states[1,i,:,j] = result

				# ################# # 


		np.save("Solution_Mat.npy",solution_matrix)
		np.save("Joint_State_Solutions.npy",resultant_joint_states)		

	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

	embed()

if __name__=='__main__':
	main()

