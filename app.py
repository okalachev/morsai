#!/usr/bin/env python3

import rospy
import atexit
import time
from speak import speak_ai as speak
import util

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from sensor_msgs.msg import BatteryState, Imu
from mors.srv import QuadrupedCmd

from gpt_interact import prompt as ask_gpt

rospy.init_node('morsai')
rospy.loginfo('Start MORSAI')

status_pub = rospy.Publisher('/head/status', Bool, queue_size=1)
cmd_vel_pub = rospy.Publisher('/head/cmd_vel', Twist, queue_size=1)
cmd_pose_pub = rospy.Publisher('/head/cmd_pose', Twist, queue_size=1)

rospy.wait_for_service('robot_mode')
set_mode = rospy.ServiceProxy('robot_mode', QuadrupedCmd)
robot_action = rospy.ServiceProxy('robot_action', QuadrupedCmd)

cmd_vel = Twist()
cmd_pose = Twist()
battery_state = BatteryState()

SYSTEM_PROMPT = '''Ты управляешь роботом-собакой. Тебе нужно написать программу на Python для нее.
Можешь использовать любые функции Python, включая time.sleep и т. д. Только не забывай импортировать нужные модули.
Выдай в ответ только программу на Python, без форматирования и без ```.
Для управления используй следующие функции:
set_velocity(x, y, z, yaw) - установить скорость движения робота. x - это скорость вперед, y - это скорость налево, z - это скорость вверх (не задействовано), yaw — это угловая скорость в рад/с (против часовой).
sit() - сесть.
stand() - встать.
give_hand() - подать лапу.
wave_hand() - помахать лапой.
speak(text) - произнести текст.
get_battery_voltage() - получить напряжение батареи в вольтах.
get_pitch() - получить угол по тангажу в радинах.
get_roll() - получить угол по крену в радианах.
get_yaw() - получить угол по рысканью в радианах.
Считай, что функции для управления роботом уже объявлены.
Максимальная скорость движения робота по x - 0.5 м/с, а по y - 0.2 м/с, по yaw - 0.4 рад/с.
Учитывай, что 180 градусов это 3.14 радиан.
Для расчета времени поворота используй формулу в коде.
Положительная скорость по yaw - это вращение против часовой, то есть налево.
Отрицательная скорость по yaw - это вращение по часовой, то есть направо.
Напряжение батареи - это общее напряжение на 6 банках. Полная зарядка - это 4.2 В на банку, а разряженная - это 3.7 В на банку.
Код будет выполняться при помощи функции exec.
Напиши программу в соответствии с запросом пользователя:
'''

def gpt(prompt: str) -> str:
    return ask_gpt(SYSTEM_PROMPT, prompt)


def stop():
    print('stop!')
    publish_timer.shutdown()
    status_pub.publish(False)
    time.sleep(1)

rospy.on_shutdown(stop)


def battery_callback(msg):
    global battery_state
    battery_state = msg


rospy.Subscriber('/bat', BatteryState, battery_callback)


def imu_callback(msg):
    global imu_data
    imu_data = msg


rospy.Subscriber('/imu/data', Imu, imu_callback)


def get_battery_voltage():
    return battery_state.voltage


def get_pitch():
    yaw, pitch, roll = util.euler_from_orientation(imu_data.orientation)
    return pitch


def get_roll():
    yaw, pitch, roll = util.euler_from_orientation(imu_data.orientation)
    return roll


def get_yaw():
    yaw, pitch, roll = util.euler_from_orientation(imu_data.orientation)
    return yaw


def get_angular_z(yaw_rate, x):
    # radius = 1 - z https://github.com/voltdog/mors_base/blob/b90d7/locomotion_controller/scripts/zmp_controller/robot_controller.py#L467
    radius = x / yaw_rate
    z = 1 - radius
    return z


def sit():
    set_mode(2)  # control "pose"
    cmd_pose.angular.y = 0.5


def stand():
    while cmd_pose.angular.y > 0:
        cmd_pose.angular.y -= 0.01
        cmd_pose_pub.publish(cmd_pose)
        rospy.sleep(0.02)
    set_mode(0)  # control velocity


def give_hand():
    robot_action(3)


def wave_hand():
    robot_action(5)


def set_velocity(x, y, z, yaw):
    set_mode(0)
    if yaw != 0 and abs(x) < 0.1:
        x = 0.1
    cmd_vel.linear.x = x
    cmd_vel.linear.y = y
    cmd_vel.linear.z = z
    cmd_vel.angular.z = yaw * 2 # get_angular_z(yaw, x) # TODO:


def publish_cmd_vel(event):
    cmd_vel_pub.publish(cmd_vel)
    cmd_pose_pub.publish(cmd_pose)
    status_pub.publish(True)


publish_timer = rospy.Timer(rospy.Duration(1 / 50), publish_cmd_vel)


def do_command(prompt):
    program = gpt(prompt)
    print(program)
    g = {'set_velocity': set_velocity, 'speak': speak, 'get_battery_voltage': get_battery_voltage,
            'get_pitch': get_pitch, 'get_roll': get_roll, 'get_yaw': get_yaw, 'sit': sit, 'stand': stand,
            'give_hand': give_hand, 'wave_hand': wave_hand}
    exec(program, g)


def voice_command(msg):
    print('Do voice command: ', msg.data)
    do_command(msg.data)


rospy.Subscriber('/voice_command', String, voice_command)


while True:
    print('Input prompt')
    prompt = input()
    if not prompt:
        break
    do_command(prompt)
