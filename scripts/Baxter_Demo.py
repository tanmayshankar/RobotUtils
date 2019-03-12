#!/usr/bin/env python2
from Baxter_Ambidextrous_Planning import *
from Reset_Robot import RobotResetManager

def main():	
	try:

		movegroup = MoveGroupPythonInterface()
		print("Created Movegroup.")
		image_retriever = ImageRetriever()		
		print("Created Image Retriever.")
		reset_manager = RobotResetManager(movegroup)
		print("Created Robot Manager.")

		time.sleep(5)
		# Wait for the nodes to initialize and for images to be published before you try to retrieve images. 
		# # # Show frontal camera image. 
		plt.imsave("Image1.png",image_retriever.retrieve_image(1))

		print("Reset to VALID end effector pose.")
		end_eff_pose = [0.3, -0.3, 0.09798524029948213, 0.38044099037703677, 0.9228975092885654, -0.021717379118030174, 0.05525572942370394]
		reset_manager.set_to_end_effector_pose(end_eff_pose,"right")
		time.sleep(2)

		plt.imsave("Image2.png",image_retriever.retrieve_image(1))

		print("Reset to some joint angle configuration.")		
		joint_angles = list(np.random.random(7)-0.5)
		reset_manager.set_to_joint_pose(joint_angles)	
		time.sleep(2)
		plt.imsave("Image3.png",image_retriever.retrieve_image(1))

	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return


	embed()

if __name__=='__main__':
	main()
