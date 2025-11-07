import numpy as np
import matplotlib.pyplot as plt
from models import BicycleModel
from occupancy_grid import OccupancyGrid
from circle_generator import CircleGenerator
from optimizer import TrajectoryOptimizer
from scenarios import create_scenario_1, create_scenario_2, create_scenario_3

def run_scenario(scenario_func, scenario_name):
	print(f"\nЗапуск сценария: {scenario_name}")
	print("=" * 50)
	try:
		print("1. Создание сценария...")
		og, reference_path = scenario_func()
		print(f"   Создана карта {og.width}x{og.height}, точек пути: {len(reference_path)}")

		print("2. Генерация кругов...")
		generator = CircleGenerator(og, safety_margin=0.3)
		circles = generator.generate_optimized_circles(reference_path)
		print(f"   Сгенерировано {len(circles)} ограничивающих кругов")

		print("3. Настройка оптимизатора...")
		model = BicycleModel(L=2.5, dt=0.1)
		print("   Модель создана")
		optimizer = TrajectoryOptimizer(model, N=min(20, len(reference_path)), dt=0.1)
		print("   Оптимизатор создан")
		optimizer.setup_optimizer(reference_path, circles)
		print("   Оптимизатор настроен")

		print("4. Установка начального состояния...")
		x0 = np.array([reference_path[0][0], reference_path[0][1], 0.5, 0, 0])
		print(f"   Начальное состояние: {x0}")

		print("5. Решение оптимизации...")
		x_opt, u_opt = optimizer.solve(x0, reference_path, circles)

		if x_opt is not None and len(x_opt) > 0:
			print("   Оптимизация завершена успешно!")
			print(f"   Длина траектории: {len(x_opt)} точек")
			print("6. Визуализация...")
			filename = f'trajectory_{scenario_name.replace(" ", "_")}.png'
			og.visualize(path=reference_path, circles=circles[:len(x_opt)],
				trajectory=x_opt[:, :2],
				filename=filename
			)

		else:
			print("   Оптимизация не удалась")
			og.visualize(path=reference_path, circles=circles)
	except Exception as e:
		print(f"ОШИБКА в сценарии {scenario_name}: {e}")
		import traceback
		traceback.print_exc()
def main():
	print("Запуск системы планирования траектории CIAO")
	print("Используется CasADi + IPOPT")

	scenarios = [
		(create_scenario_1, "Прямоугольные препятствия"),
		(create_scenario_2, "Узкий коридор"),
		(create_scenario_3, "Сложный лабиринт")
	]

	for i, (scenario_func, scenario_name) in enumerate(scenarios):
		print(f"\n{'='*60}")
		print(f"СЦЕНАРИЙ {i+1}/{len(scenarios)}: {scenario_name}")
		print(f"{'='*60}")
		run_scenario(scenario_func, scenario_name)
		if i < len(scenarios) - 1:
			input("\nНажмите Enter для перехода к следующему сценарию...")

if __name__ == "__main__":
    main()
    print("\nВсе сценарии завершены!")
