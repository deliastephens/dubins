import matplotlib.pyplot as plt
import numpy as np
def draw_simulation(data, xf, r):
  '''
  Creates a plot for the UAV Trajectory.
  '''
  # draw parking trajectory
  plt.figure(figsize=(10, 10))
  fig  = plt.gcf()
  ax = plt.gca()

  plt.plot(data[0,:], data[1,:])
  circle1 = plt.Circle((xf[0], xf[2]), r, color='r', fill=False)

  ax.add_patch(circle1)

  # misc plot settings
  plt.gca().set_aspect('equal')
  # plt.legend()
  plt.xlabel(r'$x_1$')
  plt.ylabel(r'$x_2$')
  plt.title('UAV Circling Trajectory');

def draw_comparison(opt_data, lyap_data):
  """
  Lyap Data: np array of multiple trajectory optimizations
  """
  plt.figure(figsize=(10,10))
  for x_traj in lyap_data:
    x_traj[ x_traj==0 ] = np.nan
    plt.plot(x_traj[0,:], x_traj[1,:], alpha = 0.5, linewidth=1.75)
  plt.plot(opt_data[0,:], opt_data[1,:], 'b', linewidth=3)

  # misc plot settings
  plt.gca().set_aspect('equal')
  # plt.legend()
  plt.xlabel(r'$x_1$')
  plt.ylabel(r'$x_2$')
  plt.title('UAV Circling Trajectory Comparison');


def plot_input(u,close_i, time_scale=1):
  """
  Plots one input trajectory.
  u: input trajectory from a simulation (n, 1)
  """
  n_points = np.shape(u)[0]
  plt.figure(figsize=(10, 2))
  fig  = plt.gcf()
  ax = plt.gca()
  t = np.linspace(0, n_points * 0.1, n_points)

  plt.plot(t[:close_i], u[:close_i])

  plt.xlabel(r'$timesteps$')
  plt.ylabel(r'$u_t$')
  plt.title('Control Input until Circling');

def plot_inputs(us, close_is, time_scale=1):
  """
  Plots many input trajectory.
  us: a list of input trajectories
  close_is: a list of the indices to "circling"
  """
  plt.figure(figsize=(10, 4))
  fig  = plt.gcf()
  ax = plt.gca()
  for i in range(len(us)):
    u = us[i]
    close_i = int(close_is[i])
    n_points = np.shape(u)[0]
    t = np.linspace(0, n_points * 0.1, n_points)

    plt.plot(t[:close_i], u[:close_i], linewidth=2)
  plt.xlabel(r'$timesteps$')
  plt.ylabel(r'$u_t$')
  plt.title('Control Input until Circling');
    

