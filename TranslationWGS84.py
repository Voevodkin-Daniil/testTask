import math
from enum import Enum
from typing import Tuple


class EncodingDatum(Enum):
    WGS_84 = 1  # WGS-84
    GSK_2011 = 2  # ГСК-2011
    PZ_90 = 3  # ПЗ-90.11
    SK = 4  # СК-42 / СК-95 (эллипсоид Красовского)


class TranslationWGS84:
    """ГОСТ 32453-2017 - Преобразования координат между различными датумами"""

    # Параметры эллипсоидов
    _ELLIPSOID_PARAMS = {
        EncodingDatum.WGS_84: (6378137.0, 1.0 / 298.257223563),
        EncodingDatum.GSK_2011: (6378136.5, 1.0 / 298.2564151),
        EncodingDatum.PZ_90: (6378136.0, 1.0 / 298.25784),
        EncodingDatum.SK: (6378245.0, 1.0 / 298.3)
    }

    @classmethod
    def _get_ellipsoid_params(cls, datum: EncodingDatum) -> Tuple[float, float]:
        """Возвращает параметры референц-эллипсоида для заданной системы координат"""
        if datum not in cls._ELLIPSOID_PARAMS:
            raise ValueError(f"Неизвестная система координат: {datum}")
        return cls._ELLIPSOID_PARAMS[datum]

    # Преобразование геодезических <=> декартовых координат

    @classmethod
    def convert_geodetic_to_cartesian(cls,
                                      latitude: float,
                                      longitude: float,
                                      datum: EncodingDatum,
                                      altitude: float = 0) -> Tuple[float, float, float]:
        """
        Преобразует геодезические координаты (широта, долгота, высота)
        в геоцентрические прямоугольные координаты (X, Y, Z)

        Args:
            latitude: Геодезическая широта в радианах
            longitude: Геодезическая долгота в радианах
            datum: Геодезический датум
            altitude: Высота над эллипсоидом в метрах (по умолчанию 0)

        Returns:
            Кортеж (X, Y, Z) в метрах
        """
        a, f = cls._get_ellipsoid_params(datum)
        e2 = 2 * f - f * f  # квадрат первого эксцентриситета
        sin_lat = math.sin(latitude)
        N = a / math.sqrt(1 - e2 * sin_lat * sin_lat)

        x = (N + altitude) * math.cos(latitude) * math.cos(longitude)
        y = (N + altitude) * math.cos(latitude) * math.sin(longitude)
        z = ((1 - e2) * N + altitude) * sin_lat

        return (x, y, z)

    @classmethod
    def convert_geodetic_to_cartesian_tuple(cls,
                                            geodetic: Tuple[float, float],
                                            datum: EncodingDatum,
                                            altitude: float = 0) -> Tuple[float, float, float]:
        """Перегрузка для работы с кортежем (B, L)"""
        return cls.convert_geodetic_to_cartesian(geodetic[0], geodetic[1], datum, altitude)

    @classmethod
    def convert_cartesian_to_geodetic(cls,
                                      x: float, y: float, z: float,
                                      datum: EncodingDatum) -> Tuple[float, float, float]:
        """
        Преобразует геоцентрические прямоугольные координаты (X, Y, Z)
        в геодезические координаты (широта, долгота, высота)

        Args:
            x, y, z: Координаты в метрах
            datum: Геодезический датум

        Returns:
            Кортеж (B, L, H) - широта (рад), долгота (рад), высота (м)
        """
        a, f = cls._get_ellipsoid_params(datum)
        e2 = 2 * f - f * f
        eps = 1e-10  # допуск итерации

        p = math.sqrt(x * x + y * y)  # расстояние до оси Z

        # Случай точки на оси вращения
        if p < eps:
            lat2 = math.pi / 2 if z >= 0 else -math.pi / 2
            lon2 = 0
            sin_lat = math.sin(lat2)
            N = a / math.sqrt(1 - e2 * sin_lat * sin_lat)
            h2 = z * sin_lat - N * (1 - e2 * sin_lat * sin_lat)
            return (lat2, lon2, h2)

        # Долгота
        lon = math.atan2(y, x)
        if lon < 0:
            lon += 2 * math.pi

        # Итеративное вычисление широты по ГОСТ (алгоритм 5.1.2)
        r = math.sqrt(p * p + z * z)
        c = math.asin(z / r)
        p1 = e2 * a / (2 * r)

        s1 = 0
        while True:
            b = c + s1
            s2 = math.asin(p1 * math.sin(2 * b) / math.sqrt(1 - e2 * math.pow(math.sin(b), 2)))
            lat = b
            if abs(s2 - s1) < eps:
                break
            s1 = s2

        # Высота
        sin_lat_fin = math.sin(lat)
        N_fin = a / math.sqrt(1 - e2 * sin_lat_fin * sin_lat_fin)
        h = p * math.cos(lat) + z * sin_lat_fin - N_fin * (1 - e2 * sin_lat_fin * sin_lat_fin)

        return (lat, lon, h)

    @classmethod
    def convert_cartesian_to_geodetic_tuple(cls,
                                            cartesian: Tuple[float, float, float],
                                            datum: EncodingDatum) -> Tuple[float, float, float]:
        """Перегрузка для работы с кортежем (X, Y, Z)"""
        return cls.convert_cartesian_to_geodetic(cartesian[0], cartesian[1], cartesian[2], datum)

    # Преобразования между датумами (по ГОСТ 32453-2017)

    @classmethod
    def convert_pz90_to_wgs84(cls, pz90: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """
        Преобразует геоцентрические координаты из системы ПЗ-90.11 в WGS-84(G1150)

        Args:
            pz90: Кортеж (X, Y, Z) в системе ПЗ-90.11 (метры)

        Returns:
            Кортеж (X, Y, Z) в системе WGS-84 (метры)
        """
        x, y, z = pz90
        return (
            x + (+2.041066e-8) * y + (+1.716240e-8) * z - (-0.003),
            (-2.041066e-8) * x + y + (+1.115071e-8) * z - (-0.001),
            (-1.716240e-8) * x + (-1.115071e-8) * y + z - (0.000)
        )

    @classmethod
    def convert_wgs84_to_pz90(cls, wgs84: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """
        Преобразует геоцентрические координаты из системы WGS-84(G1150) в ПЗ-90.11

        Args:
            wgs84: Кортеж (X, Y, Z) в системе WGS-84 (метры)

        Returns:
            Кортеж (X, Y, Z) в системе ПЗ-90.11 (метры)
        """
        x, y, z = wgs84
        return (
            x + (-2.041066e-8) * y + (-1.716240e-8) * z + (-0.003),
            (+2.041066e-8) * x + y + (-1.115071e-8) * z + (+0.001),
            (+1.716240e-8) * x + (+1.115071e-8) * y + z + (0.000)
        )

    @classmethod
    def convert_pz90_to_gsk2011(cls, pz90: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """
        Преобразует геоцентрические координаты из системы ПЗ-90.11 в ГСК-2011

        Args:
            pz90: Кортеж (X, Y, Z) в системе ПЗ-90.11 (метры)

        Returns:
            Кортеж (X, Y, Z) в системе ГСК-2011 (метры)
        """
        x, y, z = pz90
        return (
            x + (-2.56951e-10) * y + (-9.21146e-11) * z - (0.000),
            (+2.569513e-10) * x + y + (-2.72465e-9) * z - (+0.014),
            (+9.211460e-11) * x + (-2.72465e-9) * y + z - (-0.008)
        )

    @classmethod
    def convert_gsk2011_to_pz90(cls, gsk2011: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """
        Преобразует геоцентрические координаты из системы ГСК-2011 в ПЗ-90.11

        Args:
            gsk2011: Кортеж (X, Y, Z) в системе ГСК-2011 (метры)

        Returns:
            Кортеж (X, Y, Z) в системе ПЗ-90.11 (метры)
        """
        x, y, z = gsk2011
        return (
            x + (+2.569513e-10) * y + (+9.211460e-11) * z + (0.000),
            (-2.569513e-10) * x + y + (+2.724653e-9) * z + (+0.014),
            (-9.211460e-11) * x + (-2.724653e-9) * y + z + (-0.008)
        )

    @classmethod
    def convert_pz90_to_sk(cls, pz90: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """
        Преобразует геоцентрические координаты из системы ПЗ-90.11 в СК-42/95 (эллипсоид Красовского)

        Args:
            pz90: Кортеж (X, Y, Z) в системе ПЗ-90.11 (метры)

        Returns:
            Кортеж (X, Y, Z) в системе СК (метры)
        """
        x, y, z = pz90
        return (
            x + (+6.506684e-7) * y + (+1.716240e-8) * z - (+24.457),
            (-6.506684e-7) * x + y + (+1.115071e-8) * z - (-130.784),
            (-1.716240e-8) * x + (-1.115071e-8) * y + z - (-81.538)
        )

    @classmethod
    def convert_sk_to_pz90(cls, sk: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """
        Преобразует геоцентрические координаты из системы СК-42/95 (эллипсоид Красовского) в ПЗ-90.11

        Args:
            sk: Кортеж (X, Y, Z) в системе СК (метры)

        Returns:
            Кортеж (X, Y, Z) в системе ПЗ-90.11 (метры)
        """
        x, y, z = sk
        return (
            x + (-6.506684e-7) * y + (-1.716240e-8) * z + (+24.457),
            (+6.506684e-7) * x + y + (-1.115071e-8) * z + (-130.784),
            (+1.716240e-8) * x + (+1.115071e-8) * y + z + (-81.538)
        )

    # Проекция Гаусса-Крюгера (эллипсоид Красовского)

    @classmethod
    def convert_to_gauss_krueger(cls, latitude_wgs84: float, longitude_wgs84: float) -> Tuple[float, float]:
        """
        Преобразует геодезические координаты WGS-84 в плоские прямоугольные координаты
        проекции Гаусса-Крюгера (зональная система, эллипсоид Красовского)

        Args:
            latitude_wgs84: Геодезическая широта в WGS-84 (радианы)
            longitude_wgs84: Геодезическая долгота в WGS-84 (радианы)

        Returns:
            Кортеж (x, y) в метрах, где x - северное смещение (ордината),
            y - восточное смещение с номером зоны
        """
        # Цепочка: WGS84 (геод.) -> декартовы WGS84 -> PZ-90 -> SK -> геод. SK
        wgs_cart = cls.convert_geodetic_to_cartesian(latitude_wgs84, longitude_wgs84, EncodingDatum.WGS_84)
        pz_cart = cls.convert_wgs84_to_pz90(wgs_cart)
        sk_cart = cls.convert_pz90_to_sk(pz_cart)
        sk_geo = cls.convert_cartesian_to_geodetic(sk_cart[0], sk_cart[1], sk_cart[2], EncodingDatum.SK)
        B, L, _ = sk_geo  # широта и долгота в радианах (эллипсоид Красовского)

        # Прямая проекция Гаусса-Крюгера (формулы 24-26 ГОСТ)
        L_deg = L * 180.0 / math.pi
        n = int((6 + L_deg) / 6)
        L0 = (3 + 6 * (n - 1)) * math.pi / 180.0  # осевой меридиан в радианах
        l = L - L0

        sinB = math.sin(B)
        sin2B = math.sin(2 * B)
        sinB2 = sinB * sinB
        sinB4 = sinB2 * sinB2
        sinB6 = sinB4 * sinB2
        l2 = l * l

        # Вычисление x по формуле (24)
        term1 = 16002.8900 + 66.9607 * sinB2 + 0.3515 * sinB4
        term2 = 1594561.25 + 5336.535 * sinB2 + 26.790 * sinB4 + 0.149 * sinB6
        term3 = 672483.4 - 811219.9 * sinB2 + 5420.0 * sinB4 - 10.6 * sinB6
        term4 = 278194 - 830174 * sinB2 + 572434 * sinB4 - 16010 * sinB6
        term5 = 109500 - 574700 * sinB2 + 863700 * sinB4 - 398600 * sinB6

        x = 6367558.4968 * B - sin2B * (term1 - l2 * (term2 + l2 * (term3 + l2 * (term4 + l2 * term5))))

        # Вычисление y по формуле (25)
        y = (5 + 10 * n) * 100000 + l * math.cos(B) * (
                6378245 + 21346.1415 * sinB2 + 107.1590 * sinB4 + 0.5977 * sinB6 +
                l2 * (1070204.16 - 2136826.66 * sinB2 + 17.98 * sinB4 - 11.99 * sinB6 +
                      l2 * (270806 - 1523417 * sinB2 + 1327645 * sinB4 - 21701 * sinB6 +
                            l2 * (79690 - 866190 * sinB2 + 1730360 * sinB4 - 945460 * sinB6)))
        )

        return (x, y)

    @classmethod
    def convert_from_gauss_krueger(cls, x: float, y: float) -> Tuple[float, float, float]:
        """
        Преобразует плоские прямоугольные координаты проекции Гаусса-Крюгера
        (зональная система, эллипсоид Красовского) в геодезические координаты WGS-84

        Args:
            x: Северное смещение (ордината) в метрах
            y: Восточное смещение с номером зоны в метрах (первые цифры - номер зоны)

        Returns:
            Кортеж (B, L, H) - широта (рад), долгота (рад), высота (м) в системе WGS-84
        """
        # Обратная проекция (формулы 29-36 ГОСТ)
        n = int(y * 1e-6)
        beta = x / 6367558.4968

        # Вычисление B0 по формуле (32)
        sin_beta = math.sin(beta)
        sin_beta2 = sin_beta * sin_beta
        sin_beta4 = sin_beta2 * sin_beta2
        sin2_beta = math.sin(2 * beta)

        B0 = beta + sin2_beta * (0.00252588685 - 0.00001491860 * sin_beta2 + 0.00000011904 * sin_beta4)

        # Теперь используем B0 для вычисления ΔB и l
        sinB0 = math.sin(B0)
        sinB02 = sinB0 * sinB0
        sinB04 = sinB02 * sinB02
        sinB06 = sinB04 * sinB02
        sin2B0 = math.sin(2 * B0)
        cosB0 = math.cos(B0)

        z0 = (y - (10 * n + 5) * 100000) / (6378245 * cosB0)
        z02 = z0 * z0

        # Формула (33) для ΔB
        dB = -z02 * sin2B0 * (
                0.251684631 - 0.003369263 * sinB02 + 0.00001127 * sinB04 -
                z02 * (0.10500614 - 0.04559916 * sinB02 + 0.00228901 * sinB04 - 0.00002987 * sinB06 -
                       z02 * (0.042858 - 0.025318 * sinB02 + 0.014346 * sinB04 - 0.001264 * sinB06 -
                              z02 * (0.01672 - 0.00630 * sinB02 + 0.01188 * sinB04 - 0.00328 * sinB06)))
        )

        # Формула (34) для l
        l = z0 * (
                1 - 0.0033467108 * sinB02 - 0.0000056002 * sinB04 - 0.0000000187 * sinB06 -
                z02 * (0.16778975 + 0.16273586 * sinB02 - 0.00052490 * sinB04 - 0.00000846 * sinB06 -
                       z02 * (0.0420025 + 0.1487407 * sinB02 + 0.0059420 * sinB04 - 0.0000150 * sinB06 -
                              z02 * (0.01225 + 0.09477 * sinB02 + 0.03282 * sinB04 - 0.00034 * sinB06 -
                                     z02 * (0.0038 + 0.0524 * sinB02 + 0.0482 * sinB04 - 0.0032 * sinB06))))
        )

        # Геодезические координаты в системе СК (эллипсоид Красовского)
        B_sk = B0 + dB
        L_sk = (6 * (n - 0.5)) * math.pi / 180.0 + l

        # Обратная цепочка: СК (геод.) -> декартовы СК -> PZ-90 -> WGS84 -> геод. WGS84
        sk_cart = cls.convert_geodetic_to_cartesian(B_sk, L_sk, EncodingDatum.SK)
        pz_cart = cls.convert_sk_to_pz90(sk_cart)
        wgs_cart = cls.convert_pz90_to_wgs84(pz_cart)
        wgs_geo = cls.convert_cartesian_to_geodetic(wgs_cart[0], wgs_cart[1], wgs_cart[2], EncodingDatum.WGS_84)

        return wgs_geo  # (B, L, H) - широта (рад), долгота (рад), высота (м)

    # Координаты локальной касательной плоскости

    @classmethod
    def convert_geodetic_to_local_tangent(cls,
                                          target_latitude: float,
                                          target_longitude: float,
                                          target_altitude: float,
                                          origin_latitude: float,
                                          origin_longitude: float,
                                          origin_altitude: float) -> Tuple[float, float, float]:
        """
        Преобразует геодезические координаты WGS-84 в локальные плоские координаты
        (восток, север, вверх) относительно опорной точки

        Args:
            target_latitude: Широта целевой точки в радианах
            target_longitude: Долгота целевой точки в радианах
            target_altitude: Высота целевой точки в метрах
            origin_latitude: Широта опорной точки (начала координат) в радианах
            origin_longitude: Долгота опорной точки (начала координат) в радианах
            origin_altitude: Высота опорной точки (начала координат) в метрах

        Returns:
            Кортеж (east, north, up) - локальные плоские координаты в метрах
        """
        # Преобразуем целевую точку в ECEF-прямоугольные координаты
        target_cart = cls.convert_geodetic_to_cartesian(
            target_latitude, target_longitude, EncodingDatum.WGS_84, target_altitude
        )

        # Преобразуем опорную точку в ECEF-прямоугольные координаты
        origin_cart = cls.convert_geodetic_to_cartesian(
            origin_latitude, origin_longitude, EncodingDatum.WGS_84, origin_altitude
        )

        # Вычисляем разность координат (перевод начала координат в опорную точку)
        dx = target_cart[0] - origin_cart[0]
        dy = target_cart[1] - origin_cart[1]
        dz = target_cart[2] - origin_cart[2]

        # Вычисляем матрицу вращения для перехода к локальной касательной плоскости
        sin_lat = math.sin(origin_latitude)
        cos_lat = math.cos(origin_latitude)
        sin_lon = math.sin(origin_longitude)
        cos_lon = math.cos(origin_longitude)

        # Матрица преобразования из ECEF в локальные координаты (восток, север, вверх)
        east = -sin_lon * dx + cos_lon * dy
        north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
        up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz

        return (east, north, up)

    @classmethod
    def convert_local_tangent_to_geodetic(cls,
                                          east: float,
                                          north: float,
                                          up: float,
                                          origin_latitude: float,
                                          origin_longitude: float,
                                          origin_altitude: float) -> Tuple[float, float, float]:
        """
        Преобразует локальные плоские координаты (восток, север, вверх)
        обратно в геодезические координаты WGS-84

        Args:
            east: Восточное смещение в метрах
            north: Северное смещение в метрах
            up: Вертикальное смещение в метрах
            origin_latitude: Широта опорной точки (начала координат) в радианах
            origin_longitude: Долгота опорной точки (начала координат) в радианах
            origin_altitude: Высота опорной точки (начала координат) в метрах

        Returns:
            Кортеж (B, L, H) - широта (рад), долгота (рад), высота (м) в системе WGS-84
        """
        # Преобразуем опорную точку в ECEF-прямоугольные координаты
        origin_cart = cls.convert_geodetic_to_cartesian(
            origin_latitude, origin_longitude, EncodingDatum.WGS_84, origin_altitude
        )

        # Вычисляем матрицу обратного преобразования (транспонированная матрица прямого преобразования)
        sin_lat = math.sin(origin_latitude)
        cos_lat = math.cos(origin_latitude)
        sin_lon = math.sin(origin_longitude)
        cos_lon = math.cos(origin_longitude)

        # Обратное преобразование из локальных координат в ECEF
        dx = -sin_lon * east - sin_lat * cos_lon * north + cos_lat * cos_lon * up
        dy = cos_lon * east - sin_lat * sin_lon * north + cos_lat * sin_lon * up
        dz = cos_lat * north + sin_lat * up

        # Добавляем координаты опорной точки
        x = dx + origin_cart[0]
        y = dy + origin_cart[1]
        z = dz + origin_cart[2]

        # Преобразуем обратно в геодезические координаты WGS-84
        return cls.convert_cartesian_to_geodetic(x, y, z, EncodingDatum.WGS_84)

    @classmethod
    def convert_geodetic_to_local_tangent_tuple(cls,
                                                target: Tuple[float, float, float],
                                                origin: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Перегрузка для работы с кортежами (B, L, H)"""
        return cls.convert_geodetic_to_local_tangent(
            target[0], target[1], target[2],
            origin[0], origin[1], origin[2]
        )

    @classmethod
    def convert_local_tangent_to_geodetic_tuple(cls,
                                                local: Tuple[float, float, float],
                                                origin: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Перегрузка для работы с кортежами"""
        return cls.convert_local_tangent_to_geodetic(
            local[0], local[1], local[2],
            origin[0], origin[1], origin[2]
        )

    # Методы используемые для тестов

    @classmethod
    def calculate_distance_in_local_tangent(cls,
                                            point1: Tuple[float, float, float],
                                            point2: Tuple[float, float, float]) -> float:
        """
        Вычисляет расстояние между двумя точками в локальной касательной плоскости

        Args:
            point1: Координаты первой точки (B, L, H) в радианах и метрах
            point2: Координаты второй точки (B, L, H) в радианах и метрах

        Returns:
            Расстояние в метрах
        """
        # Используем первую точку как начало координат
        local = cls.convert_geodetic_to_local_tangent_tuple(point2, point1)

        # Вычисляем евклидово расстояние в локальной плоскости (игнорируем вертикальную компоненту)
        return math.sqrt(local[0] * local[0] + local[1] * local[1])

    @classmethod
    def calculate_azimuth_in_local_tangent(cls,
                                           from_point: Tuple[float, float, float],
                                           to_point: Tuple[float, float, float]) -> float:
        """
        Вычисляет азимут (угол от севера) от первой точки ко второй в локальной касательной плоскости

        Args:
            from_point: Исходная точка (B, L, H) в радианах и метрах
            to_point: Целевая точка (B, L, H) в радианах и метрах

        Returns:
            Азимут в радианах от 0 до 2π (0 = север, π/2 = восток)
        """
        local = cls.convert_geodetic_to_local_tangent_tuple(to_point, from_point)

        # Азимут измеряется от севера по часовой стрелке
        azimuth = math.atan2(local[0], local[1])
        if azimuth < 0:
            azimuth += 2 * math.pi

        return azimuth