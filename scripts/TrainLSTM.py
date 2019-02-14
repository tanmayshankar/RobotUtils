#!/usr/bin/env python2

import sys, copy, rospy, moveit_commander, numpy as np
import moveit_msgs.msg, geometry_msgs.msg
from math import pi
import torch
from IPython import embed
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from Baxter_Planning import MoveGroupPythonInterface

class LSTMNetwork(torch.nn.Module):

	# Policy Network inherits from torch.nn.Module. 
	# Now we overwrite the init, forward functions. And define anything else that we need. 

	def __init__(self, input_size, hidden_size, output_size):

		# Ensures inheriting from torch.nn.Module goes nicely and cleanly. 	
		super().__init__()

		self.input_size = input_size
		self.hidden_size = hidden_size
		self.output_size = output_size
		self.num_layers = 1
		self.maximum_length = 15
		self.state_size = 2
		
		# Create LSTM Network. 
		self.lstm = torch.nn.LSTM(input_size=self.input_size,hidden_size=self.hidden_size,num_layers=self.num_layers)		
		# Define output layers for the LSTM, and activations for this output layer. 
		self.output_layer = torch.nn.Linear(self.hidden_size, self.output_size)		
		# Stopping probability predictor. (Softmax, not sigmoid)
		self.stopping_probability_layer = torch.nn.Linear(self.hidden_size, 2)	

		self.softmax_layer = torch.nn.Softmax(dim=)

	def forward(self, input):
		# Here, input is just the initial and final states between which we predict a path. 
		format_input = torch.tensor(input).view(input.shape[0],1,self.input_size).float()
		input_zeros = torch.zeros_like(format_input)
		t=0
		stop = False
		hidden = None
		states = []
		states.append(format_input[0,0,:self.state_size])

		# self.input_size must be 2 x State_Size.

		while not(stop):
			# Forward pass the actual LSTM, remember, this is now a single timestep output. 
			if t==0:							
				# Feed in actual start and stop state. 
				lstm_output, hidden = self.lstm(format_input)
			else:
				# Feed in zeros (and hidden).
				lstm_output, hidden = self.lstm(input_zeros, hidden)

			# Increment counter.
			t+=1 

			# lstm_output should be shape T x 1 x Hidden_Size
			# Get action outputs
			state_output = self.output_layer(lstm_output[0])
			# Get stopping probability
			probability = self.softmax_layer(self.stopping_probability_layer(lstm_output[0]))

			# Should we be stopping? 
			if t>=self.maximum_length:
				stop = True
			else:
				stop = self.sample_action(probability)

		# Would've appended intended goal state but this may not match what the network predicts. 			
		# states.append(format_input[0,0,self.state_size:])


	def forward(self, input, action_sequence):
		# Input is the trajectory sequence of shape: Sequence_Length x 1 x Input_Size. 
		format_input = torch.tensor(input).view(input.shape[0],1,self.input_size).float()

		format_action_seq = torch.from_numpy(action_sequence).cuda().float().view(action_sequence.shape[0],1,self.output_size)
		lstm_outputs, hidden = self.lstm(format_input)

		# Predict Gaussian means and variances. 
		mean_outputs = self.activation_layer(self.mean_output_layer(lstm_outputs))
		variance_outputs = self.softplus_activation_layer(self.variances_output_layer(lstm_outputs))

		# Remember, because of Pytorch's dynamic construction, this distribution can have it's own batch size. 
		# It doesn't matter if batch sizes changes over different forward passes of the LSTM, because we're only going
		# to evaluate this distribution (instance)'s log probability with the same sequence length. 
		dist = torch.distributions.MultivariateNormal(mean_outputs, torch.diag_embed(variance_outputs))
		log_probabilities = dist.log_prob(format_action_seq)
		# log_probabilities = torch.distributions.MultivariateNormal(mean_outputs, torch.diag_embed(variance_outputs)).log_prob(format_action_seq)
		entropy = dist.entropy()
		return log_probabilities, entropy

	def sample_action(self, action_probabilities):
		# Categorical distribution sampling. 
		sample_action = torch.distributions.Categorical(probs=action_probabilities).sample().squeeze(0)
		return sample_action

	def select_greedy_action(self, action_probabilities):
		# Select action with max probability for test time. 
		return action_probabilities.argmax()

	def select_epsilon_greedy_action(self, action_probabilities):
		epsilon = 0.1
		if np.random.random()<epsilon:
			return self.sample_action(action_probabilities)
		else:
			return self.select_greedy_action(action_probabilities)

def main():
	try:
		movegroup = MoveGroupPythonInterface()

		# joint_goal = movegroup.group.get_current_joint_values()
		# joint_goal[0] += 0.2
		# joint_goal[2] += 0.2
		# joint_goal[6] += 0.2
		# plan = movegroup.go_to_joint_state(joint_goal)	
		
		plan = movegroup.go_to_pose_goal()		
		plan_array = movegroup.parse_plan(plan)
		
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
	main()