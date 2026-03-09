from Multirotor import Multirotor
import time


def main():
    if __name__ == "__main__":
        try:
            BP = Multirotor()

            print("\n=== Статус дрона ===")
            status = BP.get_status()
            print(f"Текущая позиция: {status['position']}")
            print(f"Домашняя позиция: {status['home']}")
            print(f"Расстояние до дома: {status['distance_to_home']:.2f}м")

            # Выбор режима GUIDED
            if not BP.set_mode('GUIDED'):
                print("Не удалось установить режим GUIDED")
                exit(1)

            # Армирование
            if not BP.arm():
                print("Не удалось выполнить армирование")
                exit(1)

            # Взлет
            if not BP.takeoff(7):
                print("Не удалось выполнить взлет")
                exit(1)

            # Полет по прямоугольному маршруту
            print("\n=== Миссия: Прямоугольный маршрут ===")

            # Вперед и вправо (диагональ)
            if not BP.flight_to(100, 300):
                print("Не удалось долететь до точки 1")

            # Назад по горизонтали
            if not BP.flight_to(-100, 0):
                print("Не удалось долететь до точки 2")

            # Возврат домой
            print("\n=== Возвращение домой ===")
            if not BP.home():
                print("Не удалось вернуться домой, пробую прямой режим RTL...")
                BP.set_mode('RTL')
                time.sleep(15)

            print("\n=== Миссия завершена ===")
            print(f"Конечная позиция: lat={BP.lat:.7f}°, lon={BP.lon:.7f}°, alt={BP.alt:.1f}м")

        except ConnectionError as e:
            print(f"Ошибка подключения: {e}")
        except KeyboardInterrupt:
            print("\nПрограмма прервана пользователем")
            # Попытка безопасного завершения
            if 'BP' in locals():
                print("Попытка вернуться домой и сесть...")
                BP.set_mode('RTL')
                time.sleep(5)
        except Exception as e:
            print(f"Непредвиденная ошибка: {e}")
            import traceback

            traceback.print_exc()


if __name__ == "__main__":
    main()