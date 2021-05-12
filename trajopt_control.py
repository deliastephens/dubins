# python libraries
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.patches as patches

# pydrake imports
from pydrake.systems.scalar_conversion import TemplateSystem
from pydrake.all import (Variable, SymbolicVectorSystem, VectorSystem, DiagramBuilder,
                         LogOutput, Simulator, ConstantVectorSource,
                         MathematicalProgram, Solve, SnoptSolver, PiecewisePolynomial, eq, cos, sin)
from pydrake.systems.framework import (BasicVector, BasicVector_, LeafSystem_,
                                       LeafSystem)
import pydrake.symbolic as sym

from visualization import *


"""
Trajectory Optimization Dynamics.
Continuous and Discrete Dubins Dynamics for the Trajectory Optimization Problem
"""
def dubins_continuous_dynamics(x, u):
	m = sym if x.dtype == object else np # Check type for autodiff
	# x = x, y, yaw
	# u = steering angle
	yaw = x[2]

	x_dot = np.array([
		m.cos(yaw),
		m.sin(yaw),
		u[0]
		])

	return x_dot


def dubins_discrete_dynamics(x, x_next, u, time_step):
	'''
	Residuals of the Dubins discre-time dynamics
	If the vector of residuals is zero, then this method's arguments verify the
	discrete time dynamics.
	Uses an implict Euler formulation.
	'''
	x_dot = dubins_continuous_dynamics(x_next, u)
	residuals = x_next - x - time_step * x_dot

	return residuals


 ###
# Trajectory optimization helper functions
###
def interpolate_dubins_state(x0, xf, time_steps, time_interval):
	'''
	Creates an initial guess for the state by drawing a straight line between
	the initial and final positions
	'''
	np.random.seed(0)

	# initial and final time and state
	time_limits = [0., time_steps * time_interval]
	position_limits = np.column_stack((x0, xf))
	state_limits = position_limits

	# linear interpolation in state
	state = PiecewisePolynomial.FirstOrderHold(time_limits, state_limits)

	# sample state on the time grid and add small random noise
	state_guess = np.vstack([state.value(t * time_interval).T for t in range(time_steps + 1)])
	state_guess += np.random.rand(*state_guess.shape) * 5e-6

	print(np.shape(state_guess))

	return state_guess

def constraint_state_to_orbit(x, x_next, xf, r, time_step):
	'''
	If the vector of residuals is zero, then the state of the Dubins car is
	"orbiting" around the given final position.

	Calculates these residuals by confirming theere is zero radial velocity,
	and the final position is r away from the desired orbit position.
	'''
	# unpack state, rocket position in relative coordinates
	p = x[:2] - xf[:2]
	v = (x_next[:2] - x[:2]) / time_step

	# constraint on radial distance
	# sets x^2 + y^2 to the orbit radius squared
	residual_p = p.dot(p) - r ** 2

	# radial velocity must be zero
	# sets the time derivative of x^2 + y^2 to zero
	residual_v = p.dot(v)

	# gather constraint residuals
	residuals = np.array([residual_p])

	return residuals

###
# Trajectory optimization setup
###

def trajopt_simulation(x0, xf, u_max=0.5):
	# numeric parameters
	time_interval = .1
	time_steps = 400

	# initialize optimization
	prog = MathematicalProgram()

	# optimization variables
	state = prog.NewContinuousVariables(time_steps + 1, 3, 'state')
	u = prog.NewContinuousVariables(time_steps, 1, 'u')

	# final position constraint
	for residual in constraint_state_to_orbit(state[-2], state[-1], xf, 1/u_max, time_interval):
		prog.AddConstraint(residual == 0)

	# initial position constraint
	prog.AddConstraint(eq(state[0],x0))

	# discretized dynamics
	for t in range(time_steps):
		residuals = dubins_discrete_dynamics(state[t], state[t+1], u[t], time_interval)
		for residual in residuals:
			prog.AddConstraint(residual == 0)

		# control limits
		prog.AddConstraint(u[t][0] <= u_max)
		prog.AddConstraint(-u_max <= u[t][0])

		# cost - increases cost if off the orbit
		x = state[t][0]
		y = state[t][1]
		prog.AddCost((x-xf[0])**2 + (y-xf[1])**2 - (1/u_max)**2)


	# initial guess
	state_guess = interpolate_dubins_state(
		np.array([0, 0, np.pi]),
		xf,
		time_steps, 
		time_interval
		)

	prog.SetInitialGuess(state, state_guess)

	solver = SnoptSolver()
	result = solver.Solve(prog)

	assert result.is_success()

	# retrieve optimal solution
	u_opt = result.GetSolution(u).T
	state_opt = result.GetSolution(state).T

	draw_simulation(state_opt)

	return u_opt, state_opt