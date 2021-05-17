import matplotlib.pyplot as plt
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
  plt.figure(figsize=(10,10))

  plt.plot(opt_data[0,:], opt_data[1,:], 'b')
  plt.plot(lyap_data[0,:], lyap_data[1,:], 'r')

  # misc plot settings
  plt.gca().set_aspect('equal')
  # plt.legend()
  plt.xlabel(r'$x_1$')
  plt.ylabel(r'$x_2$')
  plt.title('UAV Circling Trajectory');