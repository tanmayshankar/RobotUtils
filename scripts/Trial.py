#!/usr/bin/env python2
from Baxter_Ambidextrous_Planning import *

def main():
	movegroup = MoveGroupPythonInterface()
	image_retriever = ImageRetriever()
	embed()

if __name__=='__main__':
	main()
