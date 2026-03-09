# PyMavlink GCS flight managment of ArduCopter SITL simulation - Up and Down

from pymavlink import mavutil
import time
import math

from TranslationWGS84 import TranslationWGS84 as WGS84


# Define flight mode
def set_mode(mode):
    # Get mode ID
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    while True:
        ack = master.recv_match(type='HEARTBEAT', blocking=True)
        if ack.custom_mode == mode_id:
            print(f"Mode changed to {mode}")
            break
        time.sleep(0.5)


# Arm vehicle
def arm():
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Vehicle armed")


# Takeoff
def takeoff(altitude):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        altitude
    )
    print(f"Takeoff command sent to {altitude} m")

    # Wait until vehicle reaches 90% of target altitude


    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        print(msg)
        current_alt = msg.relative_alt / 1000.0
        print(f"Altitude: {current_alt:.1f} m")

        if current_alt >= altitude * 0.9:
            print("Reached takeoff altitude")
            return

        time.sleep(1)


# Flight to dx [m], dy [m], duration [seconds]
def flight_to(dx, dy, duration):
    print("Starting the flight")

    # Получение текущего местоположения в качестве центра круга
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    center_lat = msg.lat / 1e7
    center_lon = msg.lon / 1e7
    alt = msg.relative_alt / 1000.0

    earth_radius = 6378137.0  # метров
    start_time = time.time()

    while time.time() - start_time < duration:
        # Направление относительно Севера
        msg = master.recv_match(type='VFR_HUD', blocking=True)
        heading = msg.heading  # градусы, 0–359
        print(f"Heading (from North): {heading}°  ", end="\r", flush=True)

        time.time() - start_time

        # Convert meters → lat/lon
        dlat = (dy / earth_radius) * (180 / math.pi)
        dlon = (dx / (earth_radius * math.cos(math.radians(center_lat)))) * (180 / math.pi)

        target_lat = center_lat + dlat
        target_lon = center_lon + dlon

        master.mav.set_position_target_global_int_send(
            0,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,  # position enabled
            int(target_lat * 1e7),
            int(target_lon * 1e7),
            alt,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

        time.sleep(0.1)

    print("")
    print("Flight to the point is completed")


def flight_to2_with_check(dx, dy,target_altitude=0, target_radius=0.5):
    """
    Летит в точку и останавливается при достижении цели.

    Args:
        dx: смещение по долготе в метрах (восток)
        dy: смещение по широте в метрах (север)
        target_radius: радиус цели в метрах (при достижении останавливаемся)
        target_altitude: целевая высота в метрах (относительная)
    """
    print("Starting the flight to relative point with target check")

    # Получение текущего местоположения
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    center_lat = msg.lat / 1e7  # в градусах
    center_lon = msg.lon / 1e7  # в градусах
    start_alt = msg.relative_alt / 1000.0  # текущая высота

    print(f"Current position: lat={center_lat:.7f}°, lon={center_lon:.7f}°, alt={start_alt:.1f}m")

    # Конвертируем градусы в радианы для расчетов
    center_lat_rad = math.radians(center_lat)
    center_lon_rad = math.radians(center_lon)

    final_lat_rad, final_lon_rad, final_alt = WGS84.convert_local_tangent_to_geodetic(
        dx, dy, target_altitude,  # east, north, up (смещения)
        center_lat_rad,  # origin latitude в радианах
        center_lon_rad,  # origin longitude в радианах
        start_alt  # origin altitude
    )

    # Конвертируем обратно в градусы для MAVLink
    final_lat = math.degrees(final_lat_rad)
    final_lon = math.degrees(final_lon_rad)

    # Используем целевую высоту, а не стартовую
    target_alt = target_altitude + start_alt

    print(f"Target position: lat={final_lat:.7f}°, lon={final_lon:.7f}°, alt={target_alt:.1f}m")

    # Проверка расстояния до цели (для отладки)
    # Конвертируем текущую позицию в радианы для расчета расстояния
    distance = WGS84.calculate_distance_in_local_tangent(
        (math.radians(center_lat), math.radians(center_lon), start_alt),
        (final_lat_rad, final_lon_rad, final_alt)
    )
    print(f"Distance to target: {distance:.2f}m")

    iteration = 0
    try:
        while True:
            # Получаем текущую позицию
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if msg:
                new_lat = msg.lat / 1e7  # градусы
                new_lon = msg.lon / 1e7  # градусы
                new_alt = msg.relative_alt / 1000.0

                # Конвертируем в радианы для расчета расстояния
                new_lat_rad = math.radians(new_lat)
                new_lon_rad = math.radians(new_lon)

                # Проверяем достижение цели
                distance = WGS84.calculate_distance_in_local_tangent(
                    (new_lat_rad, new_lon_rad, new_alt),
                    (final_lat_rad, final_lon_rad, final_alt)
                )
                print(f"Distance: {distance:.2f} m")

                if distance < target_radius:
                    print(f"Target reached! Distance: {distance:.2f}m < {target_radius}m")
                    break

            # Отправка команды на полет
            master.mav.set_position_target_global_int_send(
                0,
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b0000111111111000,  # маска: использовать только позицию
                int(final_lat * 1e7),  # широта в градусах * 1e7
                int(final_lon * 1e7),  # долгота в градусах * 1e7
                target_alt,  # целевая высота (не start_alt!)
                0, 0, 0,  # скорости
                0, 0, 0,  # ускорения
                0, 0  # yaw, yaw_rate
            )

            iteration += 1
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nFlight interrupted by user")
    except Exception as e:
        print(f"\nError during flight: {e}")

    print("Flight to the point is completed")



# Land
def land():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    print("Landing...")


    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_alt = msg.relative_alt / 1000.0
        print(f"Altitude: {current_alt:.1f} m")

        if current_alt <= 1:
            print("Landed")
            return

        time.sleep(1)



# Connect via MAVProxy UDP port (default 14550)
master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
master.wait_heartbeat()
print("Connected to MAVProxy/SITL")


# Явное указание для получения телематрии
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    1,
    1
)


set_mode('GUIDED')
arm()

takeoff(7)

#flight_to2_with_check(0,100, 10)
#flight_to2_with_check(0,100, -10)


flight_to2_with_check(0,50)
flight_to2_with_check(50,0)
flight_to2_with_check(0,-100)
flight_to2_with_check(-50,50)
land()

