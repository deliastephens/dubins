import matplotlib.pyplot as plt
def draw_simulation(data):
  '''
  Creates a plot for the UAV Trajectory.
  '''
  # draw parking trajectory
  plt.figure(figsize=(10, 10))

  plt.plot(data[0,:], data[1,:])

  # misc plot settings
  plt.gca().set_aspect('equal')
  # plt.legend()
  plt.xlabel(r'$x_1$')
  plt.ylabel(r'$x_2$')
  plt.title('UAV Circling Trajectory');