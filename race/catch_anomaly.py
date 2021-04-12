import rospy
import sys
import random

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import PositionTarget, State, ExtendedState
from geographic_msgs.msg import GeoPointStamped
from std_msgs.msg import String

from mavros_msgs.srv import SetMode, CommandBool, CommandVtolTransition, CommandHome

instances_num = 6
data = {}

# Стартовая функция для подписывания на топики
def subscribe_on_topics():
    # глобальная (GPS) система координат
    subscribe_on_mavros_topics("global_position/global", NavSatFix)

    # локальная система координат, точка отсчета = место включения аппарата
    subscribe_on_mavros_topics("local_position/pose", PoseStamped)
    subscribe_on_mavros_topics("local_position/velocity_local", TwistStamped)

    # состояние
    subscribe_on_mavros_topics("state", State)
    subscribe_on_mavros_topics("extended_state", ExtendedState)




# Подписываемся на Mavros топики всех аппаратов
def subscribe_on_mavros_topics(suff, data_class):
    for n in range(1, instances_num + 1):
        data[n] = {'wall_num': 0}
        topic = f"/mavros{n}/{suff}"
        rospy.Subscriber(topic, data_class, topic_cb, callback_args=(n, suff))




# При публикаци нового сообщения дронов
def topic_cb(msg, callback_args):
    n, suff = callback_args
    data[n][suff] = msg

drone = 1
if len(sys.argv) > 1:
    drone = int(sys.argv[1])
rospy.init_node('watcher' + str(random.randint(0,1000000)))
subscribe_on_topics()

while True:
    telemetry = data[drone].get('local_position/pose')
    if telemetry is None or not (-20 < telemetry.pose.position.x < 140 and -20 < telemetry.pose.position.y < 140 and \
        0 < telemetry.pose.position.z < 21):
        print(telemetry)
    else:
        print()
