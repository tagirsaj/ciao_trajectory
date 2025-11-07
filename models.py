import casadi as ca
import numpy as np

class BicycleModel:
	def __init__(self, L=2.5, dt =0.1):
		self.L = L
		self.dt = dt
		self.x = ca.MX.sym('x')
		self.y = ca.MX.sym('y')
		self.v = ca.MX.sym('v')
		self.theta = ca.MX.sym('theta')
		self.delta = ca.MX.sym('delta')
		self.states = ca.vertcat(self.x, self.y, self.v, self.theta, self.delta)
		self.a =ca.MX.sym('a')
		self.omega = ca.MX.sym('omega')
		self.controls = ca.vertcat(self.a, self.omega)
	def get_dynamics(self):
		x_dot = self.v * ca.cos(self.theta)
		y_dot = self.v *ca.sin(self.theta)
		v_dot = self.a
		theta_dot = self.v * ca.tan(self.delta) / self.L
		delta_dot = self.omega
		rhs = ca.vertcat(x_dot, y_dot, v_dot, theta_dot, delta_dot)
		x_next = self.states +self.dt * rhs
		return ca.Function('dynamics', [self.states, self.controls], [x_next])
class DifferentialDriveModel:
	def __init__(self, dt=0.1):
		self.dt = dt
		self.x = ca.MX.sym('x')
		self.y = ca.MX.sym('y')
		self.theta = ca.MX.sym('theta')
		self.states = ca.vertcat(self.x, self.y, self.theta)
		self.v_left = ca.MX.sym('v_left')
		self.v_right = ca.MX.sym('v_right')
		self.controls = ca.vertcat(self.v_left,self.v_right)
	def get_dynamics(self):
		R = 0.1
		L = 0.5
		v = (self.v_right + self.v_left) * R /2
		omega = (self.v_right - self.v_left) * R / L
		x_dot = v * ca.cos(self.theta)
		y_dot = v* ca.sin(self.theta)
		theta_dot = omega
		rhs = ca.vertcat(x_dot, y_dot, theta_dot)
		x_next = self.states +self.dt * rhs

		return ca.Function('dynamics', [self.states, self.controls], [x_next])

