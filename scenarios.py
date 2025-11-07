import numpy as np
from occupancy_grid import OccupancyGrid
def create_scenario_1():
	og = OccupancyGrid(width=80, height=60, resolution=0.1)

# Добавляем препятствия
	og.add_obstacle_rectangle(3, 2, 7, 3)
	og.add_obstacle_rectangle(3, 5, 7, 6)
	og.add_obstacle_circle(2, 4, 0.8)
# Опорный путь
	path = []
	for i in range(20):
		x = i * 0.4
		y = 4.0 + 0.8 * np.sin(i * 0.3)
		path.append([x, y])

	return og, np.array(path)
def create_scenario_2():

	og = OccupancyGrid(width=60, height=80, resolution=0.1)
# Создаем узкий коридор
	og.add_obstacle_rectangle(0, 0, 3, 25)
	og.add_obstacle_rectangle(5, 0, 8, 25)
# Опорный путь по центру коридора
	path = []
	for i in range(20):
		x = 4.0 + 0.3 * np.sin(i * 0.2)
		y = i * 1.0
		path.append([x, y])
	return og, np.array(path)
def create_scenario_3():
	og = OccupancyGrid(width=100, height=100, resolution=0.1)
# Сложные препятствия
	og.add_obstacle_rectangle(2, 2, 8, 3)
	og.add_obstacle_rectangle(6, 4, 7, 8)
	og.add_obstacle_rectangle(3, 6, 5, 9)
	og.add_obstacle_circle(8, 6, 1.2)
# Извилистый путь
	path = []
	for i in range(25):
		x = 1 + i * 0.3
		y = 5 + 2 * np.sin(i * 0.4)
		path.append([x, y])

	return og, np.array(path)
