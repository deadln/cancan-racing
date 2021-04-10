#!/usr/bin/env python3
# coding=utf8

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

instances_num = 3 #количество аппаратов
freq = 20 #Герц, частота посылки управляющих команд аппарату
node_name = "offboard_node"
data = {} # Словарь с топиками дронов. Ключ - номер дрона от 1 до n. Значением является некий объект через который можно обращаться к топикам посредством data[n].get('topic_name')
lz = {}
current_track_data = {}

EPS = 0.22

# Расстояние между точками
def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2) + math.pow(z1 - z2, 2))

# Преобразование списка строки с координатами в списки точек
def str_to_coords_lists(s):
    s = s.split('\n')
    for i in range(len(s)):
        s[i] = s[i].split()
        for j in range(len(s[i])):
            s[i][j] = float(s[i][j])
    return s[:-1]

# Получение центральной точки препятствия в координатах симулятора
def obstacle_to_coords(c_x, c_y, c_z, x, y):  # Координаты точки центральной линии, параметры отверстия
    return {'x': c_x, 'y': c_y - x, 'z': c_z + y}
        
# Получение нормализованного вектора, направленного в сторону стены
def get_wall_norm_vect(cent_line):
    vect = {'x': cent_line[-3] - cent_line[-6], 'y': cent_line[-2] - cent_line[-5], 'z':  cent_line[-1] - cent_line[-4]}
    vect_len = math.sqrt(pow(vect['x'], 2) + pow(vect['y'], 2) + pow(vect['z'], 2))
    vect['x'] = vect['x'] / vect_len
    vect['y'] = vect['y'] / vect_len
    vect['z'] = vect['z'] / vect_len
    return vect


def subscribe_on_mavros_topics(suff, data_class):
  #подписываемся на Mavros топики всех аппаратов
  for n in range(1, instances_num + 1):
    data[n] = {}
    topic = f"/mavros{n}/{suff}"
    rospy.Subscriber(topic, data_class, topic_cb, callback_args = (n, suff))

def subscribe_on_mavros_topics_track(topic):
  #current_track_data[topic] = []
  rospy.Subscriber(topic, String, topic_cb_track, callback_args = (topic))

def topic_cb(msg, callback_args):
  n, suff = callback_args
  data[n][suff] = msg

def topic_cb_track(msg, topic):
    #print('**topic_cb_track**')
    #current_track_data[topic].append(msg)
    current_track_data[topic] = msg

def service_proxy(n, path, arg_type, *args, **kwds):
  service = rospy.ServiceProxy(f"/mavros{n}/{path}", arg_type)
  ret = service(*args, **kwds)

  #rospy.loginfo(f"{n}: {path} {args}, {kwds} => {ret}")

def arming(n, to_arm):
  d = data[n].get("state")
  if d is not None and d.armed != to_arm:
    service_proxy(n, "cmd/arming", CommandBool, to_arm)

def set_mode(n, new_mode):
  d = data[n].get("state")
  if d is not None and d.mode != new_mode:
    service_proxy(n, "set_mode", SetMode, custom_mode=new_mode)


def subscribe_on_topics():
  # глобальная (GPS) система координат
  subscribe_on_mavros_topics("global_position/global", NavSatFix)

  #локальная система координат, точка отсчета = место включения аппарата
  subscribe_on_mavros_topics("local_position/pose", PoseStamped)
  subscribe_on_mavros_topics("local_position/velocity_local", TwistStamped)

  #состояние
  subscribe_on_mavros_topics("state", State)
  subscribe_on_mavros_topics("extended_state", ExtendedState)

  # Трасса профессиональной лиги
  subscribe_on_mavros_topics_track("/path_generator/central")
  subscribe_on_mavros_topics_track("/path_generator/walls")


def on_shutdown_cb():
  rospy.logfatal("shutdown")

#ROS/Mavros работают в системе координат ENU(Восток-Север-Вверх), автопилот px4 и протокол сообщений Mavlink используют систему координат NED(Север-Восток-Вниз)
#см. также описание mavlink сообщения https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED

#Управление по точкам, локальная система координат.
def set_pos(pt, x, y, z):
  pt.type_mask = pt.IGNORE_VX | pt.IGNORE_VY | pt.IGNORE_VZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | pt.IGNORE_YAW | pt.IGNORE_YAW_RATE

  #Смещение на восток
  pt.position.x = x
  #Смещение на север
  pt.position.y = y
  #Высота, направление вверх
  pt.position.z = z

#Управление по скоростям, локальная система координат, направления совпадают с оными в глобальной системе координат
def set_vel(pt, vx, vy, vz):
  pt.type_mask = pt.IGNORE_PX | pt.IGNORE_PY | pt.IGNORE_PZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | pt.IGNORE_YAW | pt.IGNORE_YAW_RATE

  #Скорость, направление на восток
  pt.velocity.x = vx
  #Скорость, направление на север
  pt.velocity.y = vy
  #Скорость, направление вверх
  pt.velocity.z = vz

def mc_takeoff(pt, n, dt):
  if dt<10:
    #скорость вверх
    set_vel(pt, 0, 0, 4)

    #армимся и взлетаем с заданной скоростью
    if dt>5:
      arming(n, True)

# В ЭТОЙ ФУНКЦИИ МЫ БУДЕМ ЗАДАВАТЬ ПЕРЕМЕЩЕНИЕ КОПТЕРА В ЗАВИСИМОСТИ ОТ ПЕРЕДАННОЙ ЦЕЛИ
def mc_race(pt, n, dt, target):  # Повторяется с частотой freq
  mc_takeoff(pt, n, dt)

  if dt>10 and dt<15:
    #скорость вверх
    set_vel(pt, 0, 0, 1)

  #летим в точку target
  if dt>15:
    #set_vel(pt, 5, 0, (n-2)/2)
    set_pos(pt, target['x'], target['y'], target['z'])

  #первый коптер летит по квадрату, остальные следуют с такой же горизонтальнй скоростью как первый
  #if dt>20 and dt<30:
    #set_pos(pt, 0, 5, 10)
    #if n == 1:
    #  set_pos(pt, 0, 5, 10)
      #if dt>20:
      #  set_vel(pt, 0, -5, 0)

      #if dt>24:
      #  set_vel(pt, -5, 0, 0)

      #if dt>27:
      #  set_vel(pt, 0, 5, 0)
    #else:
      #v1 = data[1]["local_position/velocity_local"].twist.linear
      #set_vel(pt, v1.x, v1.y, 0)

  #направляем каждого в свою точку
  #if dt>30 and dt<35:
  #  set_pos(pt, 0, (n-2)*3, 10)

  #снижаем на землю
  #if dt>70:
  #  set_vel(pt, 0, 0, -1)


def offboard_loop(mode):  # Запускается один раз
  pub_pt = {}
  #создаем топики, для публикации управляющих значений
  for n in range(1, instances_num + 1):
    pub_pt[n] = rospy.Publisher(f"/mavros{n}/setpoint_raw/local", PositionTarget, queue_size=10)

  pt = PositionTarget()  # Объект, посредством которого можно задать желаемое положение дрона и желаемые вектора скорости
  #см. также описание mavlink сообщения https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
  pt.coordinate_frame = pt.FRAME_LOCAL_NED

  t0 = time.time()

  # Получение данных о центральных линиях и отверстиях (TODO: Для профессиональной лиги перенести этот блок внутрь цикла управления и получать данные из топиков)
  cent_lines = []
  with open('race/centrals.txt', 'r') as f:
    cent_lines = str_to_coords_lists(f.read())
  #print('cent_lines', cent_lines)
  
  current_obstacle = {}
  obstacles = []
  with open('race/test_ws/w1.txt', 'r') as f:
    obstacles = str_to_coords_lists(f.read())
    for n in range(1, instances_num + 1):
      current_obstacle[n] = {}
      current_obstacle[n]['obstacle'] = obstacles[random.randint(0,len(obstacles) - 1)]
      current_obstacle[n]['obstacle_num'] = 0
      current_obstacle[n]['turn'] = False


  #цикл управления
  rate = rospy.Rate(freq)
  while not rospy.is_shutdown():
    dt = time.time() - t0
    central = current_track_data.get('/path_generator/central')
    walls = current_track_data.get('/path_generator/walls')
    if central is not None and walls is not None:
      print('CENTRAL', str(central))
      print('WALLS', walls)
    #управляем каждым аппаратом централизованно
    for n in range(1, instances_num + 1):
      # В ЭТОМ ЦИКЛЕ МЫ БУДЕМ ПОЛУЧАТЬ ДАННЫЕ О ТРАССЕ И ЗАДАВАТЬ ПОЛЁТНЫЕ ЦЕЛИ
      if dt < 10:
        set_mode(n, "OFFBOARD")

      if mode == 0:
        telemetry = data[n].get('local_position/pose')  # Получение текущих координат дрона
        if telemetry is None:
          continue
        # Получение координат точки центральной линии
        if current_obstacle[n]['turn']:
          cent_line_end = cent_lines[current_obstacle[n]['obstacle_num']][3:6]
          wall_vect = {'x': 0, 'y': 0, 'z': 0}
          obstacle = {'x': cent_line_end[0], 'y': cent_line_end[1], 'z': cent_line_end[2]}
        else:
          cent_line_end = cent_lines[current_obstacle[n]['obstacle_num']][-3:]
          wall_vect = get_wall_norm_vect(cent_lines[current_obstacle[n]['obstacle_num']])
          obstacle = obstacle_to_coords(cent_line_end[0], cent_line_end[1], cent_line_end[2], current_obstacle[n]['obstacle'][0], current_obstacle[n]['obstacle'][1])
          obstacle['x'] += wall_vect['x'] * 0.5
          obstacle['y'] += wall_vect['y'] * 0.5
          obstacle['z'] += wall_vect['z'] * 0.5

        #  Получение абсолютных координат цетра отверстия
        #obstacle = obstacle_to_coords(cent_line_end[0], cent_line_end[1], cent_line_end[2], current_obstacle[n]['obstacle'][0], current_obstacle[n]['obstacle'][1])
        #obstacle['x'] += wall_vect['x'] * 0.5
        #obstacle['y'] += wall_vect['y'] * 0.5
        #obstacle['z'] += wall_vect['z'] * 0.5
        #print('TARGET', obstacle['x'], obstacle['y'], obstacle['z'])
        #print('DISTANCE', get_distance(telemetry.pose.position.x, telemetry.pose.position.y, telemetry.pose.position.z, obstacle['x'], obstacle['y'], obstacle['z']))
        # Если коптер достиг отверстия
        if get_distance(telemetry.pose.position.x, telemetry.pose.position.y, telemetry.pose.position.z, obstacle['x'], obstacle['y'], obstacle['z']) < EPS:
            #  Меняем цель на следующее отверстие
            print('TARGET CHANGE')
            if current_obstacle[n]['turn']:
                current_obstacle[n]['turn'] = False
                cent_line_end = cent_lines[current_obstacle[n]['obstacle_num']][-3:]
            else:
              current_obstacle[n]['obstacle_num'] = current_obstacle[n]['obstacle_num'] + 1
              cent_line_end = cent_lines[current_obstacle[n]['obstacle_num']][3:6]
              if len(cent_lines[current_obstacle[n]['obstacle_num']]) > 6:
                current_obstacle[n]['turn'] = True
              else:
                current_obstacle[n]['turn'] = False
            with open(f'race/test_ws/w{current_obstacle[n]["obstacle_num"] + 1}.txt', 'r') as f:
              obstacles = str_to_coords_lists(f.read())
              current_obstacle[n]['obstacle'] = obstacles[random.randint(0,len(obstacles) - 1)]
            obstacle = obstacle_to_coords(cent_line_end[0], cent_line_end[1], cent_line_end[2], current_obstacle[n]['obstacle'][0], current_obstacle[n]['obstacle'][1])
            if current_obstacle[n]['turn']:
              wall_vect = get_wall_norm_vect(cent_lines[current_obstacle[n]['obstacle_num']])
              obstacle['x'] += wall_vect['x'] * 0.5
              obstacle['y'] += wall_vect['y'] * 0.5
              obstacle['z'] += wall_vect['z'] * 0.5
        
        #wall_vect = get_wall_norm_vect(cent_lines[current_obstacle[n]['obstacle_num']])
        #obstacle['x'] += wall_vect['x'] * 0.5
        #obstacle['y'] += wall_vect['y'] * 0.5
        #obstacle['z'] += wall_vect['z'] * 0.5
        mc_race(pt, n, dt, obstacle)

      pub_pt[n].publish(pt)

    rate.sleep()

if __name__ == '__main__':
  rospy.init_node(node_name)
  rospy.loginfo(node_name + " started")

  subscribe_on_topics()

  rospy.on_shutdown(on_shutdown_cb)

  m = 0

  if len(sys.argv) > 1:
    arg = sys.argv[1]
    if arg == "vtol":
      m = 1

  try:
    offboard_loop(m)
  except rospy.ROSInterruptException:
    pass

  rospy.spin()
