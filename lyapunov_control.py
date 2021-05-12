#!/usr/bin/env/python3
import importlib
import sys
from urllib.request import urlretrieve


# python libraries
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.patches as patches

import pydrake.all

# pydrake imports
from pydrake.common.containers import namedview
from pydrake.systems.scalar_conversion import TemplateSystem
from pydrake.all import (Variable, SymbolicVectorSystem, VectorSystem, DiagramBuilder,
                         LogOutput, Simulator, ConstantVectorSource,
                         MathematicalProgram, Solve, SnoptSolver, PiecewisePolynomial, eq, cos, sin, 
                         DirectTranscription, DirectCollocation)
from pydrake.systems.framework import (BasicVector, BasicVector_, LeafSystem_,
                                       LeafSystem)
import pydrake.symbolic as sym

# Stuff I wrote
from visualization import *

###
# Cartesian Dynamics of the Dubins Car, in a SymbolicVectorSystem
###
def create_symbolic_dynamics():
	# state of the robot (in cartesian coordinates)
	x1 = Variable("x1")
	x2 = Variable("x2")
	x3 = Variable("x3")

	cartesian_state = [x1, x2, x3]

	u1 = Variable("u1") # angular velocity
	input = [u1]

	dynamics = [cos(x3), sin(x3), u1]
	uav = SymbolicVectorSystem(
		state=cartesian_state,
		input=input,
		output=cartesian_state,
		dynamics=dynamics,
	)
	return uav

###
# Lyapunov Controller
### 

# Code drawn heavily from 
# https://colab.research.google.com/github/RussTedrake/underactuated/blob/master/exercises/lyapunov/control/control.ipynb

class LyapunovController(VectorSystem):
	def __init__(self, lyapunov_controller, params, final_state):
		# 3 inputs (UAV state)
		# 1 outut (UAV inputs)
		VectorSystem.__init__(self, 3, 1)
		self.lyapunov_controller = lyapunov_controller

		# problem parameters (TODO: make changeable)
		self.params = params # a, u_max, eps
		self.fx1, self.fx2, self.fx3 = final_state
		self.r = 1/self.params[1]

	def DoCalcVectorOutput(
		self, 
		context,
		cartesian_state,
		controller_state,
		input
	):
		# unpack the UAV state
		x1, x2, x3 = cartesian_state
		ex1 = x1 - self.fx1
		ex2 = x2 - self.fx2

		# augmented states
		xb1 = ex1*np.cos(x3) + ex2*np.sin(x3)
		xb2 = -ex1*np.sin(x3) + ex2*np.cos(x3) + self.r
		xb3 = xb2 - x3
		state = np.array([xb1, xb2, xb3])
		input[:] = self.lyapunov_controller(state, self.params )

def lyapunov_controller(state, params):
	'''
	Returns the Control Lyapunov for given state and parameters

	state: UAV-centered coordinates (xb1, xb2, xb3)
	params: a, u_max, eps

	'''
	# unpack state and params
	xb1, xb2, xb3 = state
	a, u_max, eps = params

	if xb1 <= -eps:
		u1 = a
	elif xb1 >= 0:
		u1 = u_max
	else: # middle case (-eps < xb < 0)
		u1 = (u_max - a)/(1 + np.e**(1/(xb1+eps)+1/xb1))+a

	return u1

def lyapunov_simulation(
 	x0,
 	xf, 
	create_dynamics=create_symbolic_dynamics, 
	a=0.1, 
	u_max=0.5, 
	eps=2.):
	'''
	Simulates and plots our Lyapunov Controller in different scenarios.

	Parameters:
	x0: initial state (x, y, yaw)
	x1: final state (x, y, yaw)
	create_dynamics: symbolic dynamics creation function
	a, eps: lyapunov controller parameters
	u_max: control limit (u_max = 1/r)
	'''
	params = np.array([a, u_max, eps])
	uavPlant = create_dynamics()

	# construction site for our closed-loop system
	builder = DiagramBuilder()

	# add the robot to the diagram
	# the method .AddSystem() simply returns a pointer to the system
	# we passed as input, so it's ok to give it the same name
	uav = builder.AddSystem(uavPlant)

	# add the controller
	controller = builder.AddSystem(LyapunovController(lyapunov_controller, params, xf))

	# wire the controller with the system
	builder.Connect(uavPlant.get_output_port(0), controller.get_input_port(0))
	builder.Connect(controller.get_output_port(0), uavPlant.get_input_port(0))

	# add a logger to the diagram
	# this will store the state trajectory
	logger = LogOutput(uavPlant.get_output_port(0), builder)

	# complete the construction of the diagram
	diagram = builder.Build()

	# reset logger to delete old trajectories
	# this is needed if you want to simulate the system multiple times
	logger.reset()

	# set up a simulation environment
	simulator = Simulator(diagram)

	# set the initial cartesian state to a random initial position
	# try initial_state = np.random.randn(3) for a random initial state
	context = simulator.get_mutable_context()
	context.SetContinuousState(x0)

	# simulate from zero to sim_time
	# the trajectory will be stored in the logger
	sim_time = 200.
	simulator.AdvanceTo(sim_time)

	draw_simulation(logger.data())
