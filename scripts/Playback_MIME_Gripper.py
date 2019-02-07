#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Joint Position Example: file playback
"""
import argparse, sys, rospy, numpy as np
import baxter_interface, os
from scipy.interpolate import interp1d
from baxter_interface import CHECK_VERSION

def try_float(x):
	try:
		return float(x)
	except ValueError:
		return None

def clean_line(line, names):
	"""
	Cleans a single line of recorded joint positions

	@param line: the line described in a list to process
	@param names: joint name keys
	"""
	#convert the line of strings to a float or None
	line = [try_float(x) for x in line.rstrip().split(',')]
	#zip the values with the joint names
	combined = zip(names[1:], line[1:])
	#take out any tuples that have a none value
	cleaned = [x for x in combined if x[1] is not None]
	#convert it to a dictionary with only valid commands
	command = dict(cleaned)
	left_command = dict((key, command[key]) for key in command.keys()
						if key[:-2] == 'left_')
	right_command = dict((key, command[key]) for key in command.keys()
						 if key[:-2] == 'right_')
	return (command, left_command, right_command, line)

def parse_commands(line):
	# Convert from string to dictionary. 
	command = eval(line.rstrip('\n'))
	# Remove useless joints. 
	command.pop("head_nod",None)
	command.pop("head_pan",None)
	command.pop("torso_t0",None)

	left_command = dict((key, command[key]) for key in command.keys()
						if key[:-2] == 'left_')
	right_command = dict((key, command[key]) for key in command.keys()
						 if key[:-2] == 'right_')
	return (command, left_command, right_command, line)

def upsample(len_joint_angles, gripper_traj):
	old_time_range = np.arange(0,len(gripper_traj))
	# new_time_range = np.arange(0,len_joint_angles)
	new_time_range = np.linspace(0,len(gripper_traj)-1, len_joint_angles)

	interpolation_object = interp1d(old_time_range,gripper_traj,kind='nearest')
	new_gripper_data = interpolation_object(new_time_range)

	return new_gripper_data

def downsample(len_joint_angles, len_gripper_traj):
	return np.linspace(0,len_joint_angles,len_gripper_traj,dtype=int)	

def map_file(filename, loops=1):
	"""
	Loops through csv file

	@param filename: the file to play
	@param loops: number of times to loop
				  values < 0 mean 'infinite'

	Does not loop indefinitely, but only until the file is read
	and processed. Reads each line, split up in columns and
	formats each line into a controller command in the form of
	name/value pairs. Names come from the column headers
	first column is the time stamp
	"""
	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
	grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
	rate = rospy.Rate(1000)

	if grip_left.error():
		grip_left.reset()
	if grip_right.error():
		grip_right.reset()
	if (not grip_left.calibrated() and
		grip_left.type() != 'custom'):
		grip_left.calibrate()
	if (not grip_right.calibrated() and
		grip_right.type() != 'custom'):
		grip_right.calibrate()

	print("Playing back: %s" % (filename,))

	joint_file = open(filename)
	left_gripper_traj = np.loadtxt(os.path.join(os.path.split(filename)[0],'left_gripper.txt'))
	right_gripper_traj = np.loadtxt(os.path.join(os.path.split(filename)[0],'right_gripper.txt'))

	lines = joint_file.readlines()
	new_left_gripper_traj = upsample(len(lines), left_gripper_traj)
	new_right_gripper_traj = upsample(len(lines), right_gripper_traj)
	downsample_indices = downsample(len(lines),len(left_gripper_traj))
	
	###################################################
	# with open(filename, 'r') as f:
	# 	lines = f.readlines()
	# keys = lines[0].rstrip().split(',')
	###################################################

	l = 0
	# If specified, repeat the file playback 'loops' number of times
	while loops < 1 or l < loops:
		i = 0
		l += 1
		print("Moving to start position...")

		_cmd, lcmd_start, rcmd_start, _raw = parse_commands(lines[0])

		left.move_to_joint_positions(lcmd_start)
		right.move_to_joint_positions(rcmd_start)

		start_time = rospy.get_time()

		# for i in range(1,len(lines)):
		for i in np.arange(1,len(lines),10):
			print("Line:", i)

			# cmd, lcmd, rcmd, values = clean_line(values, keys)
			cmd, lcmd, rcmd, values = parse_commands(lines[i])

			if rospy.is_shutdown():
				print("\n Aborting - ROS shutdown")
				return False
			if len(lcmd):
				# left.set_joint_positions(lcmd)
				left.move_to_joint_positions(lcmd, timeout=1)
			if len(rcmd):
				# right.set_joint_positions(rcmd)
				right.move_to_joint_positions(rcmd, timeout=1)

			grip_left.command_position(new_left_gripper_traj[i])
			grip_right.command_position(new_right_gripper_traj[i])


			rate.sleep()

	return True


def main():
	"""RSDK Joint Position Example: File Playback

	Uses Joint Position Control mode to play back a series of
	recorded joint and gripper positions.

	Run the joint_recorder.py example first to create a recording
	file for use with this example. This example uses position
	control to replay the recorded positions in sequence.

	Note: This version of the playback example simply drives the
	joints towards the next position at each time stamp. Because
	it uses Position Control it will not attempt to adjust the
	movement speed to hit set points "on time".
	"""
	epilog = """Related examples:  joint_recorder.py; joint_trajectory_file_playback.py."""

	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__, epilog=epilog)
	
	parser.add_argument('-f', '--file', metavar='PATH', required=True, help='path to input file')
	parser.add_argument('-l', '--loops', type=int, default=1,help='number of times to loop the input file. 0=infinite.')
	args = parser.parse_args(rospy.myargv()[1:])

	print("Initializing node... ")
	rospy.init_node("rsdk_joint_position_file_playback")
	print("Getting robot state... ")
	rs = baxter_interface.RobotEnable(CHECK_VERSION)
	init_state = rs.state().enabled

	def clean_shutdown():
		print("\nExiting example...")
		if not init_state:
			print("Disabling robot...")
			rs.disable()
	rospy.on_shutdown(clean_shutdown)

	print("Enabling robot... ")
	rs.enable()

	map_file(args.file, args.loops)


if __name__ == '__main__':
	main()
