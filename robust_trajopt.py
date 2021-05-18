#!/usr/bin/env/python3

import numpy as np
from lyapunov_control import *


def robust_trajopt(x_opt, u_opt, x0):
	x_traj = np.zeros(np.shape(x_opt))
	u_traj = np.zeros(np.shape(u_opt))
	i = 0
	close = False
	n_points = np.shape(x_opt)[1]
	while i < n_points-1:
		if not close:
			x0_long = np.repeat(x0.reshape(3, 1), n_points, axis=1)
			differences = np.linalg.norm(x_opt[:2,:] - x0_long[:2,:], axis=0)
			idx_min_diff = np.argmin(differences)

			if differences[idx_min_diff] < 0.1:
				close = True
				i = idx_min_diff
				xf = x_opt[:,min(i+1, n_points-1)]
				uf = u_opt[min(i, n_points-2)]
			else:
				xf = x_opt[:,min(idx_min_diff+5, n_points-1)] # added a lookahead
				uf = u_opt[min(idx_min_diff+4, n_points-2)]
		else:
			xf = x_opt[:,min(i+1, n_points-1)]
			uf = u_opt[min(i, n_points-2)]

		#     plt.plot([x0[0], xf[0]], [x0[1], xf[1]], c='#0f0f0f')
		#     plt.scatter(xf[0], xf[1], c='b')
		#     plt.scatter(x0[0], x0[1], c='r')

		simulator, dat, control = lyapunov_simulation(x0, xf, uf, to_point=True)

		x0 = dat[:,-1]
		x_traj[:,i] = dat[:,-1]
		u_traj[i] = control[:,-1]

		i += 1

	return x_traj, u_traj

class OptimalController(VectorSystem):
	def __init__(self, u_opt):
		# 3 inputs (UAV state)
		# 1 outut (UAV inputs)
		VectorSystem.__init__(self, 3, 1)

		self.u_opt = u_opt
		self.i = 0
		

	def DoCalcVectorOutput(
		self, 
		context,
		cartesian_state,
		controller_state,
		input
	):
		# unpack the UAV state
		# x1, x2, x3 = cartesian_state
		# ex1 = x1 - self.fx1
		# ex2 = x2 - self.fx2
		# ex3 = x3 - self.fx3
		# u1 = controller_state

		# augmented states
		# xb1 = ex1*np.cos(x3) + ex2*np.sin(x3)
		# xb2 = -ex1*np.sin(x3) + ex2*np.cos(x3) + self.r
		# xb3 = xb2 - ex3
		# state = np.array([xb1, xb2, xb3])
		input[:] = self.u_opt

# def lyapunov_controller(state, params):
# 	'''
# 	Returns the Control Lyapunov for given state and parameters

# 	state: UAV-centered coordinates (xb1, xb2, xb3)
# 	params: a, u_max, eps

# 	'''
# 	# unpack state and params
# 	xb1, xb2, xb3 = state
# 	a, u_max, eps = params

# 	if xb1 <= -eps:
# 		u1 = a
# 	elif xb1 >= 0:
# 		u1 = u_max
# 	else: # middle case (-eps < xb < 0)
# 		u1 = (u_max - a)/(1 + np.e**(1/(xb1+eps)+1/xb1))+a

# 	return u1

def simulation(
 	x0,
 	u_opt,
 	u_max=0.5):
	'''
	Simulates and plots our Lyapunov Controller in different scenarios.

	Parameters:
	x0: initial state (x, y, yaw)
	x1: final state (x, y, yaw)
	uref: reference input
	create_dynamics: symbolic dynamics creation function
	a, eps: lyapunov controller parameters
	u_max: control limit (u_max = 1/r)
	'''
	uavPlant = create_symbolic_dynamics()

	# construction site for our closed-loop system
	builder = DiagramBuilder()

	# add the robot to the diagram
	# the method .AddSystem() simply returns a pointer to the system
	# we passed as input, so it's ok to give it the same name
	uav = builder.AddSystem(uavPlant)

	# add the controller
	controller = builder.AddSystem(OptimalController(u_opt))

	# wire the controller with the system
	builder.Connect(uavPlant.get_output_port(0), controller.get_input_port(0))
	builder.Connect(controller.get_output_port(0), uavPlant.get_input_port(0))

	# add a logger to the diagram
	# this will store the state trajectory
	logger = LogOutput(uavPlant.get_output_port(0), builder)
	control_logger = LogOutput(controller.get_output_port(0), builder)

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
	sim_time = 0.1
	simulator.AdvanceTo(0.1)
	# print('done!')


	# draw_simulation(logger.data())
	return (simulator, logger.data(), control_logger.data())
