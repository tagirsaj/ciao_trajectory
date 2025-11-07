import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt

class OccupancyGrid:
	def __init__(self, width=50, height=30, resolution=0.1):
		self.width = width
		self.height = height
		self.resolution = resolution
		self.grid = np.zeros((height, width))
	def add_obstacle_rectangle(self, x1, y1, x2, y2):

		i1 = int(y1 / self.resolution)
		j1 = int(x1 / self.resolution)
		i2 = int(y2 / self.resolution)
		j2 = int(x2 / self.resolution)

		i1 = max(0, min(i1, self.height-1))
		i2 = max(0, min(i2, self.height-1))
		j1 = max(0, min(j1, self.width-1))
		j2 = max(0, min(j2, self.width-1))

		self.grid[i1:i2, j1:j2] = 1
	def add_obstacle_circle(self, x, y, radius):
		for i in range(self.height):
			for j in range(self.width):
				world_x = j * self.resolution
				world_y = i * self.resolution
				if(world_x-x) **2 + (world_y-y)**2 <= radius**2:
					if 0 <= i < self.height and 0 <= j < self.width:
						self.grid[i, j]= 1
	def is_free(self, x, y, radius=0):
		i = int(y / self.resolution)
		j = int(x / self.resolution)

		if not (0 <= i < self.height and 0 <= j < self.width):
			return False

		if radius == 0:
			return self.grid[i, j] ==0
		check_radius = int(radius / self.resolution) + 1
		for di in range(-check_radius, check_radius + 1):
			for dj in range(-check_radius, check_radius + 1):
				i_check = i + di
				j_check = j + dj
				if (0 <= i_check < self.height and 0 <= j_check < self.width and di**2 + dj**2 <= check_radius**2):
					if self.grid[i_check, j_check] == 1:
						return False
		return True
	def visualize(self, path=None, circles=None, trajectory=None,filename=None):
		plt.figure(figsize=(12,10))
		plt.imshow(self.grid.T, cmap = 'binary', origin = 'lower', extent = [0, self.width*self.resolution, 0, self.height*self.resolution])
		if path is not None:
			plt.plot(path[:,0], path[:,1], 'r--', label="oporny put", linewidth=2)
		if circles is not None:
			for circle in circles:
				x0, y0, r =circle
				if r > 0:
					circle_plot = plt.Circle((x0, y0), r, color = 'blue', alpha = 0.3, fill = False, linestyle='--')
					plt.gca().add_patch(circle_plot)
		if trajectory is not None:
			plt.plot(trajectory[:, 0], trajectory[:, 1], 'g-',label = 'optimized trajectory', linewidth=3)
		plt.legend()
		plt.grid(True)
		plt.title("Occupancy Grid withh trajectory")
		plt.xlabel('X [m]')
		plt.ylabel('Y [m]')
		if filename is None:
			import time
			filename = f'trajectory_{int(time.time())}.png'
		plt.savefig(filename)
		print(f"Result has saved. like a...'{filename}'")
		plt.show()
