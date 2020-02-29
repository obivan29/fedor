import socket
import struct

HOST = "127.0.0.1"
PORT = 2368

NUM_LASERS = 16
LASER_ANGLES = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]

EXPECTED_SCAN_DURATION = 0.1
DISTANCE_RESOLUTION = 0.002
ROTATION_RESOLUTION = 0.01
ROTATION_MAX_UNITS = 36000


# Функция get_distance определяет минимальное расстояние (в метрах) по указанному азимуту
# Параметры:
#    azimuth - азимут, по которому необходимо определить расстояние
# Возвращаемое значение - расстояние в метрах
def get_distance(azimuth=0.0):
    # Получить облако точек по азимут +- 1.5 градуса, о одному лазеру с индексом 1 (1 градус)
    points = get_points(azimuth - 1.5, azimuth + 1.5, [1])
    distance = min(point['distance'] for point in points)

    return distance


# Функция get_points возвращает облако точек,
# у которых:
# азимут находится в диапазоне от min_azimuth (-180.0 градусов)
#    до max_azimuth (+180.0 градусов)
# идентификатор лазера в списке lasers
# Возвращает список, состоящий из записей (dict) вида
#    'laser_id' - идентификатор лазера
#    'azimuth' - азимут, в градусах [-180.0, +180.0]
#    'distance' - расстояние, в метрах
# Возвращаемый список будет отсортирован по 1: azimuth 2: laser_id
def get_points(min_azimuth=-180.0, max_azimuth=180.0, lasers=range(16)):
    points = []

    for data in get_data():
        for point in unpack_data(data):
            if not min_azimuth <= point['azimuth'] <= max_azimuth:
                continue
            if point['laser_id'] not in lasers:
                continue

            points.append(point)

    points = sorted(points, key=lambda i: (i['azimuth'], i['laser_id']))

    return points


# Получить данные лидара Velodyne VLP-16
# Возвращает упакованные данные
def get_data():
    data_set = []
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))

    for i in range(11):
        data, addr = sock.recvfrom(2048)
        data_set.append(data)

    return data_set


# Преобразовать данные, полученные от лидара, в облако точек
# Структура данных, возвращаемых лидаром, описана в VLP-16 User Manual,
# см Figure 9-2 "VLP-16 Single Return Mode Data Structure" на странице 57
# Возвращает список точек. По
def unpack_data(data):
    points = []

    for offset in range(0, 1200, 100):
        flag, azimuth = struct.unpack_from("<HH", data, offset)
        assert flag == 0xEEFF, hex(flag)

        for step in range(2):
            azimuth += step
            azimuth %= ROTATION_MAX_UNITS

            # H-distance (2mm step), B-reflectivity (0)
            pts = struct.unpack_from('<' + "HB" * 16, data, offset + 4 + step * 48)
            for i in range(NUM_LASERS):
                points.append(calc(pts[i * 2], azimuth, i))

    return points


# Пересчитать данные, возвращаемые лидаром, из единиц измерения лидара
# в градусы и метры
def calc(distance, azimuth, laser_id):
    distance *= DISTANCE_RESOLUTION

    # если угол в диапазоне (180, 360],
    # то перевести такой угол к отрицательному значению
    if azimuth > (ROTATION_MAX_UNITS / 2):
        azimuth -= ROTATION_MAX_UNITS

    azimuth *= ROTATION_RESOLUTION

    return {
        'laser_id': laser_id,
        'azimuth': azimuth,
        'distance': distance
    }


if __name__ == '__main__':
    points = get_points(-2.5, 2.5, [1])

    print('points:', len(points), 'first:', points[0]['azimuth'], 'last:', points[-1]['azimuth'])
    print(points[0]['distance'])
    print(points[-1]['distance'])

    print('distance:', get_distance())
