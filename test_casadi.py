# test_casadi.py
import casadi as ca
import numpy as np

print("Testing CasADi...")

x = ca.MX.sym('x')
y = ca.MX.sym('y')
f = (x-1)**2 + (y-2)**2
nlp = {'x': ca.vertcat(x,y), 'f': f}
solver = ca.nlpsol('solver', 'ipopt', nlp)
result = solver(x0=[0,0])
print("CasADi test passed!")
print(f"Solution: {result['x']}")
