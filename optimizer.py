import casadi as ca
import numpy as np

class TrajectoryOptimizer:
	def __init__(self, model, N=20, dt=0.1):
		self.model = model
		self.N = N
		self.dt = dt
		self.solver = None

	def setup_optimizer(self, reference_path, circles):
		print(f"Setting up optimizer with horizon {self.N}...")
		nx = self.model.states.size1()
		nu = self.model.controls.size1()
		X = ca.MX.sym('X', nx, self.N+1)
		U = ca.MX.sym('U', nu, self.N)
		X_ref = ca.MX.sym('X_ref', 5, self.N+1)
		circles_param = ca.MX.sym('circles', 3, self.N+1)
		dynamics = self.model.get_dynamics()
		g = []
		J = 0
		weights = {
			'x': 10.0, 'y': 10.0, 'v': 1.0, 'theta': 5.0, 'delta': 5.0,
			'a': 0.1, 'omega': 0.1
		}
		for k in range(self.N):
			x_next = dynamics(X[:, k], U[:, k])
			g.append(x_next - X[:, k+1])
		for k in range(self.N+1):
			circle_constraint = (X[0, k] - circles_param[0, k])**2 + (X[1, k] - circles_param[1, k])**2
			g.append(circle_constraint - circles_param[2, k]**2)
			if k < self.N:
				J += weights['x'] * (X[0, k] - X_ref[0, k])**2
				J += weights['y'] * (X[1, k] - X_ref[1, k])**2
				J += weights['v'] * (X[2, k] - X_ref[2, k])**2
				J += weights['theta'] * (X[3, k] - X_ref[3, k])**2
				J += weights['delta'] * (X[4, k] - X_ref[4, k])**2
				J += weights['a'] * U[0, k]**2
				J += weights['omega'] * U[1, k]**2
		nlp = {
			'x': ca.vertcat(X.reshape((-1, 1)), U.reshape((-1, 1))),
			'f': J,
			'g': ca.vertcat(*g),
			'p': ca.vertcat(X_ref.reshape((-1, 1)), circles_param.reshape((-1, 1)))
		}
		opts = {
			'ipopt.print_level': 0,
			'print_time': 0,
			'ipopt.tol': 1e-4
		}
		try:
			self.solver = ca.nlpsol('solver', 'ipopt', nlp, opts)
			print("Optimizer setup completed successfully")
		except Exception as e:
			print(f"Error setting up optimizer: {e}")
			self.solver = None
	def solve(self, x0, reference_path, circles):
		if self.solver is None:
			return self.solve_simple_mpc(x0, reference_path, circles)
		X_ref = np.zeros((5, self.N+1))
		circles_param = np.zeros((3, self.N+1))
		for k in range(min(self.N+1, len(reference_path))):
			X_ref[0, k] = reference_path[k][0]
			X_ref[1, k] = reference_path[k][1]
			X_ref[2, k] = 1.0
			X_ref[3, k] = 0.0
			X_ref[4, k] = 0.0
			if k < len(circles):
				circles_param[0, k] = circles[k][0]
				circles_param[1, k] = circles[k][1]
				circles_param[2, k] = circles[k][2]
		x_init = np.tile(x0, (self.N+1, 1)).T
		u_init = np.zeros((2, self.N))
		try:
			res = self.solver(
				x0=np.vstack([x_init.reshape((-1, 1)), u_init.reshape((-1, 1))]),
				p=np.vstack([X_ref.reshape((-1, 1)), circles_param.reshape((-1, 1))]),
				lbg=-1e-8,
				ubg=1e-8
			)
			x_opt = res['x'][:self.model.states.size1()*(self.N+1)]
			u_opt = res['x'][self.model.states.size1()*(self.N+1):]
			x_opt = x_opt.full().reshape((self.model.states.size1(), self.N+1))
			u_opt = u_opt.full().reshape((self.model.controls.size1(), self.N))
			return x_opt.T, u_opt.T
		except Exception as e:
			print(f"Optimization failed: {e}")
			return self.solve_simple_mpc(x0, reference_path, circles)
	def solve_simple_mpc(self, x0, reference_path, circles):
		print("Using simple MPC fallback...")
		x_opt = [x0]
		u_opt = []
		for k in range(min(self.N, len(reference_path)-1)):
			dx = reference_path[k+1][0] - x_opt[-1][0]
			dy = reference_path[k+1][1] - x_opt[-1][1]
			a = 0.1 * np.sign(dx * np.cos(x_opt[-1][3]) + dy * np.sin(x_opt[-1][3]))
			omega = 0.05 * np.arctan2(dy, dx)
			u_opt.append([a, omega])
			dynamics = self.model.get_dynamics()
			next_state = dynamics(x_opt[-1], [a, omega])
			x_opt.append(next_state.full().flatten())
		return np.array(x_opt), np.array(u_opt)
