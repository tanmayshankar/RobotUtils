#!/usr/bin/env python2
from Baxter_Ambidextrous_Planning import *

def main():	
	try:

		movegroup = MoveGroupPythonInterface()
		image_retriever = ImageRetriever()		


		# Wait for the nodes to initialize and for images to be published before you try to retrieve images. 
		time.sleep(5)
		
		# # Show frontal camera image. 
		image = image_retriever.retrieve_image(2)
		plt.imshow(image)
		plt.show()

		movegroup.left_limb.move_to_neutral()
		# Show left camera image.
		image = image_retriever.retrieve_image(3)
		plt.imshow(image)	
		plt.show()

		movegroup.right_limb.move_to_neutral()	
		# Show right camera image.
		image = image_retriever.retrieve_image(2)
		plt.imshow(image)	
		plt.show()

	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return


	embed()

if __name__=='__main__':
	main()