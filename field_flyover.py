import numpy as np
import rospy
import time
import sys
import math

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import PositionTarget, State, ExtendedState
from gazebo_msgs.msg import ModelStates

from mavros_msgs.srv import SetMode, CommandBool, CommandVtolTransition, CommandHome

node_name = 'offboard_node'
n_mc = 1
n_rover = 1
instances_num = 1
freq = 20  # Герц, частота посылки управляющих команд аппарату

data = {}
data_gazebo = ModelStates() # данные из топика моделей gazebo
routes = {}  # маршруты для каждой модели
point_counts = {}  # номера текущих точек для моделей
lz = {}  # зоны посадки
correction = {}  # веторы кореекции координат,используются из-за того, что каждая модель считает точкой (0,0) место
# появления
points = []  # все точки для объезда поля
is_landed = []  # список состояний хранящий состояния посадки моделей

EPS = 0.8  # эпсилон для приближения к точке
LINE_DISTANCE_MC = 10  # расстояние между линиями облёта для коптеров
LINE_DISTANCE_ROVER = 3  # расстояние между линиями облёта для коптеров
ALTITUDE = 10


def subscribe_on_mavros_topics(suff, data_class, instances_num, start_num):
    """Подписываться на Mavros топики всех аппаратов"""
    for n in range(1, instances_num + 1):
        data[n] = {}
        topic = f"/mavros{n}/{suff}"
        rospy.Subscriber(topic, data_class, topic_cb, callback_args=(n, suff))


def topic_cb(msg, callback_args):
    n, suff = callback_args
    data[n][suff] = msg

def topic_cb_gazebo(msg, callback_args):
    global data_gazebo
    data_gazebo = msg


def service_proxy(n, path, arg_type, *args, **kwds):
    """Вызов сервиса"""
    service = rospy.ServiceProxy(f"/mavros{n}/{path}", arg_type)
    ret = service(*args, **kwds)

    # rospy.loginfo(f"{n}: {path} {args}, {kwds} => {ret}")


def arming(n, to_arm):
    """Арминг робота"""
    d = data[n].get("state")
    if d is not None and d.armed != to_arm:
        service_proxy(n, "cmd/arming", CommandBool, to_arm)


def set_mode(n, new_mode):
    """Изменения режима работы робота"""
    d = data[n].get("state")
    if d is not None and d.mode != new_mode:
        service_proxy(n, "set_mode", SetMode, custom_mode=new_mode)


def subscribe_on_topics(instances_num, start_num=1):
    # глобальная (GPS) система координат
    subscribe_on_mavros_topics("global_position/global", NavSatFix, instances_num, start_num)

    # локальная система координат, точка отсчета = место включения аппарата
    subscribe_on_mavros_topics("local_position/pose", PoseStamped, instances_num, start_num)
    subscribe_on_mavros_topics("local_position/velocity_local", TwistStamped, instances_num, start_num)

    # состояние
    subscribe_on_mavros_topics("state", State, instances_num, start_num)
    subscribe_on_mavros_topics("extended_state", ExtendedState, instances_num, start_num)

    # состояние моделей в gazebo (необходимо для навигации ровера)
    rospy.Subscriber("/gazebo/model_states", ModelStates, topic_cb_gazebo, callback_args=())


def on_shutdown_cb():
    rospy.logfatal("shutdown")


def set_pos(pt, p, n):
    """Полёт в точку"""
    pt.type_mask = pt.IGNORE_VX | pt.IGNORE_VY | pt.IGNORE_VZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | pt.IGNORE_YAW | pt.IGNORE_YAW_RATE

    p = p - correction[n]
    # Смещение на восток
    pt.position.x = p[0]
    # Смещение на север
    pt.position.y = p[1]
    # Высота, направление вверх
    pt.position.z = p[2]


def set_vel(pt, v, n):
    """Задать вектор скорости"""
    pt.type_mask = pt.IGNORE_PX | pt.IGNORE_PY | pt.IGNORE_PZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | pt.IGNORE_YAW | pt.IGNORE_YAW_RATE

    v = v - correction[n]
    # Скорость, направление на восток
    pt.velocity.x = v[0]
    # Скорость, направление на север
    pt.velocity.y = v[1]
    # Скорость, направление вверх
    pt.velocity.z = v[2]


def set_vel_to_point(pt, n, p):
    """Задать вектор скорости для полёта в точку. Нужно для ровера, так как на нём set_pos не работает"""
    pt.type_mask = pt.IGNORE_PX | pt.IGNORE_PY | pt.IGNORE_PZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | pt.IGNORE_YAW | pt.IGNORE_YAW_RATE

    telemetry = get_telemetry(n)
    if telemetry is None:
        # Скорость, направление на восток
        pt.velocity.x = 0
        # Скорость, направление на север
        pt.velocity.y = 0
        # Скорость, направление вверх
        pt.velocity.z = 0
        return
    v = (np.array([p[0], p[1]]) - np.array([telemetry[0], telemetry[1]])) * 2
    if np.linalg.norm(v) > 10:
        v = v / np.linalg.norm(v) * 10

    # Скорость, направление на восток
    pt.velocity.x = v[0]
    # Скорость, направление на север
    pt.velocity.y = v[1]
    # Скорость, направление вверх
    pt.velocity.z = 0


def get_telemetry(n):
    """Преобразовать телеметрию в массив numpy. Так как у ровера плагины ориентации работаю нестабильно, используются
    координаты модели, получаемые от gazebo"""
    if n <= n_rover:  # Если это ровер
        model_index = 0
        for i in range(len(data_gazebo.name)):
            if data_gazebo.name[i] == f'r1_rover{n}':
                model_index = i
                break
        try:
            telemetry = data_gazebo.pose[model_index]
        except IndexError:
            return None
    else:  # Если это коптер
        telemetry = data[n].get('local_position/pose')
        if telemetry is None:
            return None
        telemetry = telemetry.pose
    telemetry = np.array([telemetry.position.x, telemetry.position.y, telemetry.position.z]) + correction[n]
    return telemetry


def mc_flight(pt, n, dt):
    """Полёт дронов"""
    if dt > 5:
        arming(n, True)
    telemetry = get_telemetry(n)
    if telemetry is None:
        return

    if n not in lz.keys():  # запоминание зоны взлёта
        lz[n] = telemetry

    if point_counts[n] >= len(routes[n]):  # возврат в зону взлёта
        set_pos(pt, lz[n], n)
    else:
        set_pos(pt, routes[n][point_counts[n]], n)  # полёт к точке маршрута


def rover_run(pt, n, dt):
    """Езда роверов"""
    arming(n, True)
    telemetry = get_telemetry(n)
    if telemetry is None:
        return

    if n not in lz.keys():  # запоминание зоны взлёта
        lz[n] = telemetry

    if point_counts[n] >= len(routes[n]):  # возврат в зону старта
        set_vel_to_point(pt, n, lz[n])
    else:
        set_vel_to_point(pt, n, routes[n][point_counts[n]])  # полёт к точке маршрута


def offboard_loop_mc():
    """Цикл управления коптерами"""
    pub_pt = {}
    global points
    points = split_points(points, n_mc)
    # создаем топики, для публикации управляющих значений, инициализиуем служебные словари и массивы
    for n in range(n_rover + 1, n_rover + n_mc + 1):
        pub_pt[n] = rospy.Publisher(f"/mavros{n}/setpoint_raw/local", PositionTarget, queue_size=10)
        routes[n] = points[n - n_rover - 1]
        point_counts[n] = 0
        is_landed.append(0)
        correction[n] = np.array([-3+2*(n - n_rover - 1), -5, 0])

    pt = PositionTarget()
    # см. также описание mavlink сообщения https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
    pt.coordinate_frame = pt.FRAME_LOCAL_NED

    t0 = time.time()

    # цикл управления
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown() and sum(is_landed) < n_mc:

        dt = time.time() - t0

        # управляем каждым аппаратом централизованно
        for n in range(n_rover + 1, n_rover + n_mc + 1):
            pt = PositionTarget()
            pt.coordinate_frame = pt.FRAME_LOCAL_NED
            set_mode(n, "OFFBOARD")
            telemetry = get_telemetry(n)
            if telemetry is None or is_landed[n - n_rover - 1] == 1:
                continue
            # Следующая точка
            if not point_counts[n] >= len(routes[n]) and point_counts[n] >= 0 and np.linalg.norm(
                    telemetry - routes[n][point_counts[n]]) < EPS:
                point_counts[n] += 1
                if point_counts[n] < len(routes[n]):
                    print("НОВАЯ ЦЕЛЬ", routes[n][point_counts[n]])
            # Возврат в зону взлёта
            if point_counts[n] >= len(routes[n]) and np.linalg.norm(telemetry - lz[n]) < 0.2:
                arming(n, False)
                is_landed[n - n_rover - 1] = 1
                print("DISARMING")
            else:
                mc_flight(pt, n, dt)
            pub_pt[n].publish(pt)

        rate.sleep()


def offboard_loop_rover():
    """Цикл управления роверами"""
    pub_pt = {}
    global points
    points = split_points(points, n_rover)
    # создаем топики, для публикации управляющих значений
    for n in range(1, n_rover + 1):
        pub_pt[n] = rospy.Publisher(f"/mavros{n}/setpoint_raw/local", PositionTarget, queue_size=10)
        routes[n] = points[n - 1]
        point_counts[n] = 0
        is_landed.append(0)
        correction[n] = np.zeros(3)

    pt = PositionTarget()
    # см. также описание mavlink сообщения https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
    pt.coordinate_frame = pt.FRAME_LOCAL_NED

    t0 = time.time()

    # цикл управления
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown() and sum(is_landed) < n_rover:

        dt = time.time() - t0

        # управляем каждым аппаратом централизованно
        for n in range(1, n_rover + 1):
            pt = PositionTarget()
            pt.coordinate_frame = pt.FRAME_LOCAL_NED
            set_mode(n, "OFFBOARD")
            telemetry = get_telemetry(n)
            if telemetry is None or is_landed[n - 1] == 1:
                continue
            # Следующая точка
            if not point_counts[n] >= len(routes[n]) and point_counts[n] >= 0 and np.linalg.norm(
                    telemetry - routes[n][point_counts[n]]) < EPS:
                point_counts[n] += 1
                if point_counts[n] < len(routes[n]):
                    print("НОВАЯ ЦЕЛЬ", routes[n][point_counts[n]])
            # Возврат в зону взлёта
            if point_counts[n] >= len(routes[n]) and np.linalg.norm(telemetry - lz[n]) < 0.5:
                arming(n, False)
                is_landed[n - 1] = 1
                print("DISARMING")
            else:
                rover_run(pt, n, dt)
            pub_pt[n].publish(pt)

        rate.sleep()


def get_points_of_flyover(corner_a, corner_b, line_distance, altitude=0):
    """Функция принимает на вход левый нижний и правый верхний углы поля (в двумерных координатах),
    расстояние между линиями облёта поля и высоту полёта. Функция возвращает список точек маршрута для облёта поля.
    Маршрут строится в виде зиг-зага вдоль оси Y."""
    res = []
    x_direction = np.array([(corner_b[0] - corner_a[0]), 0, 0])
    y_direction = np.array([0, (corner_b[1] - corner_a[1]) / abs(corner_b[1] - corner_a[1]), 0])
    corner_a.resize(3, refcheck=False)
    corner_b.resize(3, refcheck=False)
    p = np.array(corner_a) + np.array([0, 0, altitude]).astype('float64')
    c = 0
    if corner_a[1] <= corner_b[1]:
        while p[1] < corner_b[1]:
            res.append(np.array(p))
            if c == 0:
                p += x_direction
            else:
                p -= x_direction
            res.append(np.array(p))
            p += line_distance * y_direction
            c = (c + 1) % 2
        if c == 0:
            res.append(np.array([corner_a[0], corner_b[1], altitude]))
            res.append(np.array([corner_b[0], corner_b[1], altitude]))
        else:
            res.append(np.array([corner_b[0], corner_b[1], altitude]))
            res.append(np.array([corner_a[0], corner_b[1], altitude]))
    else:
        while p[1] > corner_b[1]:
            res.append(np.array(p))
            if c == 0:
                p += x_direction
            else:
                p -= x_direction
            res.append(np.array(p))
            p += line_distance * y_direction
            c = (c + 1) % 2
        if c == 0:
            res.append(np.array([corner_a[0], corner_b[1], altitude]))
            res.append(np.array([corner_b[0], corner_b[1], altitude]))
        else:
            res.append(np.array([corner_b[0], corner_b[1], altitude]))
            res.append(np.array([corner_a[0], corner_b[1], altitude]))
    return res


def split_points(points, n):
    """Разделить точки маршрута между роботами"""
    res = []
    split_size = len(points) // n
    mod = len(points) % n
    split_sizes = []
    for i in range(n):
        split_sizes.append(split_size)
        if mod > 0:
            split_sizes[i] += 1
            mod -= 1
    i = 0
    for split in range(len(split_sizes)):
        if split < len(split_sizes) - 1:
            res.append(points[i:i + split_sizes[split] + 1])
        else:
            res.append(points[i:i + split_sizes[split]])
        i += split_sizes[split]
    return res


if __name__ == '__main__':
    rospy.init_node(node_name)
    rospy.loginfo(node_name + " started")

    n_mc = int(input('Введите количество дронов '))
    n_rover = int(input('Введите количество роверов '))
    instances_num = n_mc + n_rover
    corner_a = input('Введите первый угол поля в формате x y ')
    corner_b = input('Введите второй угол поля в формате x y ')
    corner_a = corner_a.split()
    corner_b = corner_b.split()
    for i in range(2):
        corner_a[i] = float(corner_a[i])
        corner_b[i] = float(corner_b[i])
    corner_a = np.array(corner_a)
    corner_b = np.array(corner_b)

    subscribe_on_topics(instances_num)
    rospy.on_shutdown(on_shutdown_cb)

    points = get_points_of_flyover(corner_a, corner_b, LINE_DISTANCE_MC, ALTITUDE)
    try:
        offboard_loop_mc()
    except rospy.ROSInterruptException:
        pass
    print("Облёт завершён")

    points = get_points_of_flyover(corner_a, corner_b, LINE_DISTANCE_ROVER, 0)
    is_landed = []
    try:
        offboard_loop_rover()
    except rospy.ROSInterruptException:
        pass
    print("Объезд завершён")
# Если ровер остановился или резко поехал в случайную сторону
# ekf2 stop
# ekf2 start