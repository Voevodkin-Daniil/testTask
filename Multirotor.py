from pymavlink import mavutil
import time
import math
from typing import Optional, Tuple

from TranslationWGS84 import TranslationWGS84 as WGS84


def _calculate_distance_between_points(lat1: float, lon1: float, alt1: float,
                                       lat2: float, lon2: float, alt2: float) -> float:
    """
    Расчет расстояния между двумя точками

    Returns:
        Расстояние в метрах
    """
    return WGS84.calculate_distance_in_local_tangent(
        (math.radians(lat1), math.radians(lon1), alt1),
        (math.radians(lat2), math.radians(lon2), alt2)
    )


class Multirotor:
    """
    Класс для управления мультироторным БПЛА через MAVLink
    """

    # Атрибуты
    lat: float = 0.0  # текущая широта в градусах
    lon: float = 0.0  # текущая долгота в градусах
    alt: float = 0.0  # текущая высота в метрах
    home_lat: float = 0.0  # домашняя широта в градусах
    home_lon: float = 0.0  # домашняя долгота в градусах
    home_alt: float = 0.0  # домашняя высота в метрах
    master = None

    # Константы
    HOME_RADIUS = 1.0  # радиус считающийся домом в метрах
    POSITION_TIMEOUT = 0.5  # таймаут получения позиции

    def __init__(self, connection_string: str = 'tcp:127.0.0.1:5762'):
        """
        Инициализация подключения к БПЛА

        Args:
            connection_string: строка подключения

        Raises:
            ConnectionError: если не удалось получить heartbeat
        """
        self.master = mavutil.mavlink_connection(connection_string)

        # Таймаут 3 секунды
        heartbeat = self.master.wait_heartbeat(timeout=3)
        if not heartbeat:
            raise ConnectionError(
                "Таймаут - heartbeat не получен. Убедитесь что SITL запущен и доступен по адресу " +
                connection_string)

        print("Подключено к MAVProxy")

        # Явное указание для получения телеметрии
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            1,
            1
        )

        # Небольшая пауза для получения первой телеметрии
        time.sleep(1)

        # Сохраняем домашнюю позицию (текущая позиция при инициализации)
        self._save_home_position()

    def _save_home_position(self) -> bool:
        """
        Сохранение текущей позиции как домашней

        Returns:
            True если позиция сохранена, False в противном случае
        """
        # Получаем текущую позицию
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
        if msg:
            self.home_lat = msg.lat / 1e7
            self.home_lon = msg.lon / 1e7
            self.home_alt = msg.relative_alt / 1000.0

            # Обновляем и текущую позицию
            self.lat = self.home_lat
            self.lon = self.home_lon
            self.alt = self.home_alt

            print(f"Домашняя позиция сохранена: lat={self.home_lat:.7f}°, lon={self.home_lon:.7f}°, alt={self.home_alt:.1f}м")
            return True
        else:
            print("Предупреждение: Не удалось получить начальную позицию для домашней точки")
            return False

    def _update_current_position(self) -> bool:
        """
        Обновление текущей позиции из телеметрии

        Returns:
            True если позиция обновлена, False в противном случае
        """
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            self.lat = msg.lat / 1e7
            self.lon = msg.lon / 1e7
            self.alt = msg.relative_alt / 1000.0
            return True
        return False

    def _get_current_position(self, timeout: float = 2.0) -> Optional[Tuple[float, float, float]]:
        """
        Получение текущей позиции с ожиданием

        Args:
            timeout: максимальное время ожидания в секундах

        Returns:
            Tuple (lat, lon, alt) или None
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self._update_current_position(): return self.lat, self.lon, self.alt
            time.sleep(0.1)
        return None

    def _calculate_distance_to_home(self) -> float:
        """
        Расчет расстояния до домашней позиции

        Returns:
            Расстояние до дома в метрах или бесконечность если позиция не получена
        """
        current_pos = self._get_current_position(timeout=self.POSITION_TIMEOUT)
        if not current_pos:
            return float('inf')

        current_lat, current_lon, current_alt = current_pos

        return _calculate_distance_between_points(
            current_lat, current_lon, current_alt,
            self.home_lat, self.home_lon, self.home_alt
        )

    # Define flight mode
    def set_mode(self, mode: str) -> bool:
        """
        Установка режима полета

        Args:
            mode: название режима (например, 'GUIDED', 'AUTO', 'RTL', 'LAND')

        Returns:
            True если режим успешно установлен
        """
        try:
            # Get mode ID
            mode_id = self.master.mode_mapping()[mode]
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )

            # Wait for confirmation
            timeout = time.time() + 5  # 5 seconds timeout
            while time.time() < timeout:
                ack = self.master.recv_match(type='HEARTBEAT', blocking=False)
                if ack and ack.custom_mode == mode_id:
                    print(f"Режим изменен на {mode}")
                    return True
                time.sleep(0.5)

            print(f"Таймаут ожидания смены режима на {mode}")
            return False

        except Exception as e:
            print(f"Ошибка при смене режима на {mode}: {e}")
            return False

    # Arm vehicle
    def arm(self) -> bool:
        """
        Армирование двигателей

        Returns:
            True если АРМ успешен
        """
        try:
            self.set_mode('GUIDED')

            self.master.arducopter_arm()
            self.master.motors_armed_wait()
            print("Двигатели включены")
            return True
        except Exception as e:
            print(f"Ошибка при армовании: {e}")
            return False

    def takeoff(self, altitude: float) -> bool:
        """
        Взлет на заданную высоту
        """
        if altitude <= 0:
            print("Ошибка: высота должна быть положительной")
            return False

        # Добавить проверку, не взлетел ли уже
        if hasattr(self, 'is_flying') and self.is_flying:
            print("Ошибка: дрон уже в воздухе")
            return False

        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0, 0, 0, 0, 0, 0,
                altitude
            )
            print(f"Команда взлета отправлена на высоту {altitude} м")

            # Ждем достижения высоты
            start_time = time.time()
            max_wait = 60

            while time.time() - start_time < max_wait:
                msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                if msg:
                    current_alt = msg.relative_alt / 1000.0
                    print(f"Высота: {current_alt:.1f} м")

                    if current_alt >= altitude * 0.9:
                        print("Достигнута заданная высота взлета")
                        self._update_current_position()

                        # Установить флаг, что дрон в воздухе
                        self.is_flying = True
                        self.current_altitude = current_alt
                        return True

                time.sleep(1)

            print("Таймаут взлета")
            return False

        except Exception as e:
            print(f"Ошибка во время взлета: {e}")
            return False

    def flight_to(self, dx: float, dy: float, target_altitude: float = 0, target_radius: float = 0.5) -> bool | None:
        """
        Летит в точку со смещением относительно текущей позиции и выводит сообщение о половине пути

        Args:
            dx: смещение по долготе в метрах (восток, положительное)
            dy: смещение по широте в метрах (север, положительное)
            target_altitude: целевая высота в метрах (относительная, 0 = текущая высота)
            target_radius: радиус цели в метрах

        Returns:
            True если полет выполнен успешно
        """
        print(f"Начало полета в относительную точку: dx={dx}м, dy={dy}м")

        # Получение текущего местоположения
        current_pos = self._get_current_position(timeout=3)
        if not current_pos:
            print("Ошибка: Не удалось получить текущую позицию")
            return False

        center_lat, center_lon, start_alt = current_pos

        print(f"Текущая позиция: lat={center_lat:.7f}°, lon={center_lon:.7f}°, alt={start_alt:.1f}м")

        # Конвертируем градусы в радианы для расчетов
        center_lat_rad = math.radians(center_lat)
        center_lon_rad = math.radians(center_lon)

        # Если target_altitude не указана, используем текущую высоту
        if target_altitude == 0:
            target_altitude = start_alt
        else:
            target_altitude = target_altitude  # абсолютная высота

        final_lat_rad, final_lon_rad, final_alt = WGS84.convert_local_tangent_to_geodetic(
            dx, dy, target_altitude - start_alt,  # east, north, up (смещения)
            center_lat_rad,  # origin latitude в радианах
            center_lon_rad,  # origin longitude в радианах
            start_alt  # origin altitude
        )

        # Конвертируем обратно в градусы для MAVLink
        final_lat = math.degrees(final_lat_rad)
        final_lon = math.degrees(final_lon_rad)

        print(f"Целевая позиция: lat={final_lat:.7f}°, lon={final_lon:.7f}°, alt={final_alt:.1f}м")

        # Вычисляем общее расстояние до цели
        total_distance = _calculate_distance_between_points(
            center_lat, center_lon, start_alt,
            final_lat, final_lon, final_alt
        )
        print(f"Общее расстояние до цели: {total_distance:.2f}м")

        # Флаг для отслеживания, было ли уже выведено сообщение о половине пути
        half_distance_notified = False
        stable_counter = 0  # счетчик для определения стабилизации у цели

        try:
            while True:
                # Получаем текущую позицию
                current_pos = self._get_current_position(timeout=self.POSITION_TIMEOUT)
                if not current_pos:
                    print("\rПредупреждение: Не удалось получить текущую позицию, повтор...", end="", flush=True)
                    time.sleep(0.2)
                    continue

                new_lat, new_lon, new_alt = current_pos

                # Проверяем расстояние до цели
                distance = _calculate_distance_between_points(
                    new_lat, new_lon, new_alt,
                    final_lat, final_lon, final_alt
                )

                # Выводим расстояние с перезаписью строки
                print(f"\rРасстояние до цели: {distance:.2f} м", end="", flush=True)

                # Проверка достижения цели
                if distance < target_radius:
                    stable_counter += 1
                    if stable_counter > 5:  # Подтверждаем достижение цели после 5 циклов стабильности
                        print(f"\nЦель достигнута! Расстояние: {distance:.2f}м < {target_radius}м")
                        self._update_current_position()
                        return True
                else:
                    stable_counter = 0

                # Проверка на половину пути
                if not half_distance_notified and distance <= total_distance * 0.5:
                    print(f"\n!!!!! Половина пути пройдена! Осталось {distance:.1f}м !!!!!")
                    half_distance_notified = True

                # Отправка команды на полет
                self.master.mav.set_position_target_global_int_send(
                    0,
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    0b0000111111111000,  # маска: использовать только позицию
                    int(final_lat * 1e7),  # широта в градусах * 1e7
                    int(final_lon * 1e7),  # долгота в градусах * 1e7
                    final_alt,  # целевая высота
                    0, 0, 0,  # скорости
                    0, 0, 0,  # ускорения
                    0, 0  # yaw, yaw_rate
                )

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\nПолет прерван пользователем")
            return False
        except Exception as e:
            print(f"\nОшибка во время полета: {e}")
            return False

    def home(self) -> bool | None:
        """
        Возврат в домашнюю позицию на текущей высоте, затем посадка

        Returns:
            True если возврат выполнен успешно
        """
        if self.home_lat == 0 and self.home_lon == 0:
            print("Ошибка: Домашняя позиция не установлена")
            return False

        print(f"Возврат в домашнюю позицию: lat={self.home_lat:.7f}°, lon={self.home_lon:.7f}°")

        # Получаем текущую позицию для расчета расстояния
        current_pos = self._get_current_position(timeout=3)
        if not current_pos:
            print("Ошибка: Не удалось получить текущую позицию")
            return False

        current_lat, current_lon, current_alt = current_pos
        print(f"Текущая высота: {current_alt:.1f}м, будет сохранена во время возврата")

        # Вычисляем расстояние до дома (по горизонтали)
        initial_distance = _calculate_distance_between_points(
            current_lat, current_lon, current_alt,
            self.home_lat, self.home_lon, current_alt
        )
        print(f"Горизонтальное расстояние до дома: {initial_distance:.2f}м")

        # Если мы уже рядом с домом по горизонтали, переходим сразу к посадке
        if initial_distance < self.HOME_RADIUS:
            print(f"Уже рядом с домом (расстояние: {initial_distance:.2f}м)")
            print("Переход к посадке...")
            return self.land()

        # Флаг для половины пути
        half_distance_notified = False

        try:
            while True:
                # Получаем текущую позицию
                current_pos = self._get_current_position(timeout=self.POSITION_TIMEOUT)
                if not current_pos:
                    print("\rПредупреждение: Не удалось получить текущую позицию, повтор...", end="", flush=True)
                    time.sleep(0.2)
                    continue

                new_lat, new_lon, new_alt = current_pos

                # Расчет текущего горизонтального расстояния до дома
                distance = _calculate_distance_between_points(
                    new_lat, new_lon, new_alt,
                    self.home_lat, self.home_lon, new_alt
                )

                print(f"\rРасстояние до дома: {distance:.2f} м | Высота: {new_alt:.1f}м",
                      end="", flush=True)

                # Проверка достижения дома (по горизонтали)
                if distance < self.HOME_RADIUS:
                    print(f"\nДом достигнут! Расстояние: {distance:.2f}м на высоте {new_alt:.1f}м")
                    print("Переход к посадке...")
                    return self.land()

                # Проверка на половину пути
                if not half_distance_notified and distance <= initial_distance * 0.5:
                    print(f"\n*** Половина пути до дома! Осталось {distance:.1f}м ***")
                    half_distance_notified = True

                # Отправка команды на полет домой (на текущей высоте)
                self.master.mav.set_position_target_global_int_send(
                    0,
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    0b0000111111111000,
                    int(self.home_lat * 1e7),
                    int(self.home_lon * 1e7),
                    new_alt,  # Используем ТЕКУЩУЮ высоту
                    0, 0, 0,
                    0, 0, 0,
                    0, 0
                )

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\nВозврат домой прерван пользователем")
            return False
        except Exception as e:
            print(f"\nОшибка во время возврата домой: {e}")
            return False

    def land(self) -> bool:
        """
        Посадка в текущей позиции

        Returns:
            True если посадка выполнена успешно
        """
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0,
                0, 0, 0, 0, 0, 0, 0
            )
            print("Посадка...")

            start_time = time.time()
            max_wait = 60  # maximum wait time for landing
            last_alt = float('inf')
            stuck_counter = 0

            while time.time() - start_time < max_wait:
                msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                if msg:
                    current_alt = msg.relative_alt / 1000.0
                    print(f"Высота: {current_alt:.1f} м")

                    # Проверка на застревание при посадке
                    if abs(current_alt - last_alt) < 0.05:  # менее 5 см изменения
                        stuck_counter += 1
                    else:
                        stuck_counter = 0
                        last_alt = current_alt

                    # Если застряли на высоте, пробуем разоружить
                    if stuck_counter > 20 and current_alt < 1.0:
                        print("Обнаружено зависание при посадке, разоружение...")
                        self.master.arducopter_disarm()
                        time.sleep(2)
                        return True

                    if current_alt <= 0.2:  # 0.2 meters is considered landed
                        print("Посадка завершена")
                        self._update_current_position()
                        self.is_flying = False
                        time.sleep(2)  # Wait for stabilization
                        return True

                time.sleep(1)

            print("Таймаут посадки")
            return False

        except Exception as e:
            print(f"Ошибка во время посадки: {e}")
            return False

    def get_status(self) -> dict:
        """
        Получение статуса БПЛА

        Returns:
            Словарь со статусом
        """
        self._update_current_position()
        distance_to_home = self._calculate_distance_to_home()

        return {
            'position': {'lat': self.lat, 'lon': self.lon, 'alt': self.alt},
            'home': {'lat': self.home_lat, 'lon': self.home_lon, 'alt': self.home_alt},
            'distance_to_home': distance_to_home,
            'connected': self.master is not None
        }

    def disarm(self) -> bool:
        """
        Разоружение двигателей

        Returns:
            True если разоружение успешно
        """
        try:
            self.master.arducopter_disarm()
            time.sleep(2)
            if not self.master.motors_armed():
                print("Двигатели разоружены")
                return True
            return False
        except Exception as e:
            print(f"Ошибка при разоружении: {e}")
            return False


if __name__ == "__main__":
    pass