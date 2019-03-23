#!/usr/bin/env python2
from Baxter_Ambidextrous_Planning import *
from Reset_Robot import RobotResetManager
from baxter_pykdl import baxter_kinematics
from moveit_msgs.srv import GetStateValidity

def main():	
	try:

		movegroup = MoveGroupPythonInterface()
		print("Created Movegroup.")
		image_retriever = ImageRetriever()		
		print("Created Image Retriever.")
		reset_manager = RobotResetManager(movegroup)
		print("Created Robot Manager.")
		kinematics = baxter_kinematics('right')
		print("Created Kinematics Object.")

		N = 1000
		K = 5
		dofs = 7

		joint_states = np.zeros((N,dofs))		
		end_effector_state = np.zeros((N,dofs))
		joint_state_seeds = np.zeros((N,dofs,K))

		joint_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']

		i=0
		
		# Create N joint positions to test. 
		while i<N:

			print("Currently Processing: ",i)
			# Sample a joint state. 
			sampled_state = movegroup.right_arm.get_random_joint_values()
			# Compute FK. 
			fk_result = movegroup.Compute_FK("right", movegroup.recreate_dictionary("right", sampled_state))

			# If this state is valid. 
			if fk_result.error_code.val>0:

				# Save states.
				joint_states[i] = copy.deepcopy(sampled_state)
				end_effector_state[i] = copy.deepcopy(movegroup.parse_pose(fk_result))

				# Generate seeds.
				joint_state_seeds[i] = np.random.random((dofs,K))-0.5

				i = i+1

		np.save("Joint_States.npy",joint_states)
		np.save("EE_States.npy",end_effector_state)
		np.save("Joint_State_Seeds.npy",joint_state_seeds)

	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

	embed()

if __name__=='__main__':
	main()

