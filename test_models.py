from models import BicycleModel
import numpy as np
def test_bicycle_model():

	print(" test Bicycle model...")

	model = BicycleModel()
	dynamics = model.get_dynamics()
	states = np.array([0, 0, 1.0, 0, 0])
	controls = np.array([0.1, 0.05])
	next_states = dynamics(states, controls)

	print("current status:", states)
	print("control:", controls)
	print("next status:", next_states)
	print("Model Bicycle working")

	return True

def test_differential_drive():

	print(" test Diferential drive model...")

	from models import DifferentialDriveModel

	model = DifferentialDriveModel()
	dynamics = model.get_dynamics()
	states = np.array([0, 0, 0])
	controls = np.array([1.0, 0.5])
	next_states = dynamics(states, controls)

	print("current status:", states)
	print("control:", controls)
	print("next status:", next_states)
	print("Model Diferential drive working")

	return True
if __name__ == "__main__":
	test_bicycle_model()
	test_differential_drive()

	print("all test done")
