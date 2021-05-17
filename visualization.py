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
  plt.plot(opt_data[0,:], opt_data[1,:], 'b', linewidth=1.75)
  for x_traj in lyap_data:
    x_traj[ x_traj==0 ] = np.nan
    plt.plot(x_traj[0,:], x_traj[1,:])

  # misc plot settings
  plt.gca().set_aspect('equal')
  # plt.legend()
  plt.xlabel(r'$x_1$')
  plt.ylabel(r'$x_2$')
  plt.title('UAV Circling Trajectory');