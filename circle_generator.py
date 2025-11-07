import numpy as np

class CircleGenerator:
	def __init__(self, occupancy_grid, safety_margin=0.3):
		self.og = occupancy_grid
		self.safety_margin = safety_margin
	def generate_circles_along_path(self, reference_path, max_radius=3.0, step=0.1):
		circles=[]
		for point in reference_path:
			x_ref, y_ref = point
			current_radius=step
			while current_radius <= max_radius:
				if not self.og.is_free(x_ref, y_ref, current_radius + self.safety_margin):
					break
				current_radius += step
			final_radius = max(0, current_radius - step)
			circles.append([x_ref, y_ref, final_radius])
		return circles
	def generate_optimized_circles(self, reference_path, max_radius=3.0):
		circles = []
		for i, point in enumerate(reference_path):
			x_ref, y_ref = point
			best_radius = 0
			best_center = (x_ref, y_ref)
			for dx in np.linspace(-0.5, 0.5, 5):
				for dy in np.linspace(-0.5, 0.5, 5):
					x_test = x_ref + dx
					y_test = y_ref + dy

					if not self.og.is_free(x_test, y_test, self.safety_margin):
						continue
					current_radius = 0.1
					while current_radius <= max_radius:
						if not self.og.is_free(x_test, y_test, current_radius + self.safety_margin):
							break
						current_radius += 0.1
					final_radius = max(0, current_radius - 0.1)

					if final_radius > best_radius:
						best_radius = final_radius
						best_center = (x_test, y_test)

			circles.append([best_center[0], best_center[1], best_radius])

		return circles
