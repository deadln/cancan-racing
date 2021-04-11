#!/usr/bin/env python3
# coding=utf8

# TODO:
# 1. Запилить посадку
# 2. Масштабировать алгоритм на несколько дронов
# 3. Оптимизировать проверку пролёта через препятствие посредством уравнения плоскости
# 4. Оптимизировать взлёт

import rospy
import time
import sys
import math
import random

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import PositionTarget, State, ExtendedState
from geographic_msgs.msg import GeoPointStamped
from std_msgs.msg import String

from mavros_msgs.srv import SetMode, CommandBool, CommandVtolTransition, CommandHome

instances_num = 6  # количество аппаратов
freq = 20  # Герц, частота посылки управляющих команд аппарату
node_name = "offboard_node"
# Словарь с топиками дронов. Ключ - номер дрона от 1 до n. Значением является некий объект через который можно
# обращаться к топикам посредством data[n].get('topic_name')
data = {}
current_track_data = {}
centrals = []
walls = []

current_obstacle = {}  # Словарь с текущими препятствиями для отдельных аппаратов
lz = {}

EPS = 0.22
DELAY_BETWEEN_DRONES = 2

## Вспомогательные функции

def to_points_list(points_string):
    lst = points_string.split()
    res = {'name': lst[1], 'points': []}
    i = 2
    while i < len(lst):
        res['points'].append({'x': float(lst[i]), 'y': float(lst[i + 1]), 'z': float(lst[i + 2])})
        i += 3
    return res


def to_holes_list(holes_string):
    lst = holes_string.split()
    res = {'name': lst[1], 'holes': []}
    i = 2
    while i < len(lst):
        res['holes'].append(
            {'x': float(lst[i]), 'y': float(lst[i + 1]), 'w': float(lst[i + 2]), 'h': float(lst[i + 3])})
        i += 4
    return res


# Получение центральной точки препятствия в координатах симулятора
def obstacle_to_coords(central_point, hole):  # Координаты точки центральной линии, параметры отверстия
    return {'x': central_point['x'], 'y': central_point['y'] - hole['x'], 'z': central_point['z'] + hole['y']}


# Получение нормализованного вектора, направленного в сторону стены
def get_wall_norm_vect(cent_line):
    vect = {'x': cent_line[-1]['x'] - cent_line[-2]['x'], 'y': cent_line[-1]['y'] - cent_line[-2]['y'],
            'z': cent_line[-1]['z'] - cent_line[-2]['z']}
    vect_len = math.sqrt(pow(vect['x'], 2) + pow(vect['y'], 2) + pow(vect['z'], 2))
    vect['x'] = vect['x'] / vect_len
    vect['y'] = vect['y'] / vect_len
    vect['z'] = vect['z'] / vect_len
    return vect


# Расстояние между точками
def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2) + math.pow(z1 - z2, 2))


## Функции связанные с полётом и ROS

def subscribe_on_topics():
    # глобальная (GPS) система координат
    subscribe_on_mavros_topics("global_position/global", NavSatFix)

    # локальная система координат, точка отсчета = место включения аппарата
    subscribe_on_mavros_topics("local_position/pose", PoseStamped)
    subscribe_on_mavros_topics("local_position/velocity_local", TwistStamped)

    # состояние
    subscribe_on_mavros_topics("state", State)
    subscribe_on_mavros_topics("extended_state", ExtendedState)

    # Трасса профессиональной лиги
    subscribe_on_mavros_topics_track("/path_generator/central")
    subscribe_on_mavros_topics_track("/path_generator/walls")


def subscribe_on_mavros_topics(suff, data_class):
    # подписываемся на Mavros топики всех аппаратов
    for n in range(1, instances_num + 1):
        data[n] = {'wall_num': 0}
        topic = f"/mavros{n}/{suff}"
        rospy.Subscriber(topic, data_class, topic_cb, callback_args=(n, suff))


def subscribe_on_mavros_topics_track(topic):
    rospy.Subscriber(topic, String, topic_cb_track, callback_args=(topic))


def topic_cb(msg, callback_args):
    n, suff = callback_args
    data[n][suff] = msg


def topic_cb_track(msg, topic):
    current_track_data[topic] = msg


def on_shutdown_cb():
    rospy.logfatal("shutdown")


def service_proxy(n, path, arg_type, *args, **kwds):
    service = rospy.ServiceProxy(f"/mavros{n}/{path}", arg_type)
    ret = service(*args, **kwds)

    rospy.loginfo(f"{n}: {path} {args}, {kwds} => {ret}")  # Дебаг-вывод при обращении к сервисам


def arming(n, to_arm):
    d = data[n].get("state")
    # de = data[n].get('local_position/pose')
    # print('ARMING DATA\n', de.pose.position.z)
    if d is not None and d.armed != to_arm:
        service_proxy(n, "cmd/arming", CommandBool, to_arm)


def set_mode(n, new_mode):
    d = data[n].get("state")
    if d is not None and d.mode != new_mode:
        service_proxy(n, "set_mode", SetMode, custom_mode=new_mode)


# Управление по точкам, локальная система координат.
def set_pos(pt, x, y, z):
    pt.type_mask = pt.IGNORE_VX | pt.IGNORE_VY | pt.IGNORE_VZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | pt.IGNORE_YAW | pt.IGNORE_YAW_RATE

    # Смещение на восток
    pt.position.x = x
    # Смещение на север
    pt.position.y = y
    # Высота, направление вверх
    pt.position.z = z


# Управление по скоростям, локальная система координат, направления совпадают с оными в глобальной системе координат
def set_vel(pt, vx, vy, vz):
    pt.type_mask = pt.IGNORE_PX | pt.IGNORE_PY | pt.IGNORE_PZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | pt.IGNORE_YAW | pt.IGNORE_YAW_RATE

    # Скорость, направление на восток
    pt.velocity.x = vx
    # Скорость, направление на север
    pt.velocity.y = vy
    # Скорость, направление вверх
    pt.velocity.z = vz


def mc_takeoff(pt, n, dt):
    if dt < 15:
        # скорость вверх
        set_vel(pt, 0, 0, 4)

        # армимся и взлетаем с заданной скоростью
        if dt > 5:
            arming(n, True)


def mc_race(pt, n, dt, target):  # Повторяется с частотой freq
    mc_takeoff(pt, n, dt)

    if dt > 10 and dt < 15:
        # скорость вверх
        set_vel(pt, 0, 0, 1)

    # летим в точку target
    if dt > 15 + (n - 1) * DELAY_BETWEEN_DRONES:
        #print(f'{n} is departing')
        #print('GO GO GYRO ZEPPELY')
        set_pos(pt, target['x'], target['y'], target['z'])


def get_lz(n):
    print(f'New LZ for {n}')
    # # Точка отчёта - последняя точка последней центральной линии
    # land_zone = centrals[-1]['points'][-1]
    # norm_vect = get_wall_norm_vect(centrals[-1]['points'])  # Вектор в глухую стену
    # # Сдвигаем точку до другого края посадочной площадки
    # land_zone['x'] -= norm_vect['x'] * 18.5
    # land_zone['y'] -= norm_vect['y'] * 18.5
    # # Поворачиваем вектор на 90 градусов влево
    # swp = norm_vect['x']
    # norm_vect['x'] =  norm_vect['y'] * (-1)
    # norm_vect['y'] = swp
    # # Сдвигаемся в левый нижний угол посадочной площадки
    # land_zone['x'] += norm_vect['x'] * 8.5
    # land_zone['y'] += norm_vect['y'] * 8.5
    # lz_num = len(lz)  # Номер посадочного места
    # norm_vect = get_wall_norm_vect(centrals[-1]['points'])
    # # Отсчитываем посадочное место в сторону глухой стены
    # land_zone['x'] += norm_vect['x'] * (lz_num // 8)
    # # Поворачиваем вектор на 90 градусов вправо
    # swp = norm_vect['x']
    # norm_vect['x'] = norm_vect['y']
    # norm_vect['y'] = swp * (-1)
    # # Отсчитываем посадочное место вправо
    # land_zone['y'] += norm_vect['y'] * (lz_num % 8)
    # land_zone['z'] = 1
    land_zone = {'x': 120, 'y': 120, 'z': 5}
    return land_zone

def set_target(n, telemetry):
    target = {'x': 0, 'y': 0, 'z': 0}
    #if centrals[current_obstacle[n]['wall_num']]['name'] == '|':
    #    current_obstacle[n]['landing'] = True

    # Если дрон пролетел последнее препятствие
    if current_obstacle[n]['wall_num'] >= len(walls):
        if n not in lz.keys():
            lz[n] = get_lz(n)
        target = lz[n]
        print(f'{n} got LZ at', target)
    # Если точка последняя, значит надо лететь в отверстие в стене
    elif current_obstacle[n]['point_num'] == len(centrals[current_obstacle[n]['wall_num']]['points']) - 1:
        # Если отверстие не назначено, то назначить случайное
        if 'hole_num' not in current_obstacle[n].keys():
            current_obstacle[n]['hole_num'] = random.randint(0, len(
                walls[current_obstacle[n]['wall_num']]['holes']) - 1)
        # Получаем абсолютные координаты отверстия с помощью последней точки центральной линии и локальных
        # координат отверстия
        target = obstacle_to_coords(centrals[current_obstacle[n]['wall_num']]['points'][-1],
                                    walls[current_obstacle[n]['wall_num']]['holes'][
                                        current_obstacle[n]['hole_num']])
        wall_vect = get_wall_norm_vect(centrals[current_obstacle[n]['wall_num']]['points'])
        target['x'] += wall_vect['x'] * 0.5
        target['y'] += wall_vect['y'] * 0.5
        target['z'] += wall_vect['z'] * 0.5
    # В противном случае сначала надо достигнуть точки центральной линии по пути
    else:
        target = centrals[current_obstacle[n]['wall_num']]['points'][current_obstacle[n]['point_num']]
    return target


def offboard_loop():  # Запускается один раз
    pub_pt = {}
    # создаем топики, для публикации управляющих значений
    for n in range(1, instances_num + 1):
        pub_pt[n] = rospy.Publisher(f"/mavros{n}/setpoint_raw/local", PositionTarget, queue_size=10)

    pt = PositionTarget()  # Объект, посредством которого можно задать желаемое положение дрона и желаемые вектора скорости
    # см. также описание mavlink сообщения https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
    pt.coordinate_frame = pt.FRAME_LOCAL_NED

    t0 = time.time()

    for n in range(1, instances_num + 1):
        current_obstacle[n] = {}
        current_obstacle[n]['wall_num'] = 0
        current_obstacle[n]['point_num'] = 1
        current_obstacle[n]['landing'] = False

    # цикл управления
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        dt = time.time() - t0
        central = current_track_data.get('/path_generator/central')
        wall = current_track_data.get('/path_generator/walls')
        if central is not None and wall is not None:
            central = to_points_list(str(central.data))
            wall = to_holes_list(str(wall.data))
            # print('CENTRAL', central)
            # print('WALLS', wall)
            if len(centrals) == 0 or centrals[-1]['name'] != central['name']:
                centrals.append(central)
            if wall is not None and len(walls) == 0 or walls[-1]['name'] != wall['name']:
                walls.append(wall)
        else:
            continue
        # управляем каждым аппаратом централизованно
        for n in range(1, instances_num + 1):
            # В ЭТОМ ЦИКЛЕ МЫ БУДЕМ ПОЛУЧАТЬ ДАННЫЕ О ТРАССЕ И ЗАДАВАТЬ ПОЛЁТНЫЕ ЦЕЛИ
            pt = PositionTarget()  # Объект, посредством которого можно задать желаемое положение дрона и желаемые вектора скорости
            # см. также описание mavlink сообщения https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
            pt.coordinate_frame = pt.FRAME_LOCAL_NED

            set_mode(n, "OFFBOARD")  # Переключение в режим полёта по программе

            telemetry = data[n].get('local_position/pose')  # Получение текущих координат дрона
            if telemetry is None:
                continue

            target = set_target(n, telemetry)

            if get_distance(telemetry.pose.position.x, telemetry.pose.position.y, telemetry.pose.position.z,
                            target['x'], target['y'], target['z']) < EPS:
                if current_obstacle[n]['point_num'] == len(centrals[current_obstacle[n]['wall_num']]['points']) - 1:
                    current_obstacle[n]['wall_num'] += 1
                    current_obstacle[n]['point_num'] = 1
                else:
                    current_obstacle[n]['point_num'] += 1
                target = set_target(n, telemetry)

            #print('TARGET', target)
            mc_race(pt, n, dt, target)

            pub_pt[n].publish(pt)

        rate.sleep()


if __name__ == '__main__':
    rospy.init_node(node_name)
    rospy.loginfo(node_name + " land_zoneed")

    subscribe_on_topics()

    rospy.on_shutdown(on_shutdown_cb)

    if len(sys.argv) > 1:
        instances_num = int(sys.argv[1])

try:
    offboard_loop()
except rospy.ROSInterruptException:
    pass

rospy.spin()
