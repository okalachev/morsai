#!/usr/bin/env python3

print('start')

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
from secret import OPENAI_API_KEY
from describing.describe import DescribeImage
from describing.image_processing import ImageProcessing

import change_image

print('init node')
rospy.init_node('morsai')
rospy.loginfo('Start MORSAI')

status_pub = rospy.Publisher('/head/status', Bool, queue_size=1)
cmd_vel_pub = rospy.Publisher('/head/cmd_vel', Twist, queue_size=1)
cmd_pose_pub = rospy.Publisher('/head/cmd_pose', Twist, queue_size=1)

print('wait for robot_mode')
rospy.wait_for_service('robot_mode')
set_mode = rospy.ServiceProxy('robot_mode', QuadrupedCmd)
print('wait for robot_action')
robot_action = rospy.ServiceProxy('robot_action', QuadrupedCmd)
print('done')

cmd_vel = Twist()
cmd_pose = Twist()
battery_state = BatteryState()
image_processor = ImageProcessing()
image_describer = DescribeImage(OPENAI_API_KEY)

SYSTEM_PROMPT = '''Ты управляешь роботом-собакой. Тебе нужно написать программу на Python для нее.
Можешь использовать любые функции Python, включая time.sleep и т. д. Только не забывай импортировать нужные модули.
Никогда не передавай отрицательное значение в sleep. И не делай бесконечных циклов.
Для управления используй следующие функции:
set_velocity(x, y, z, yaw) - установить скорость движения робота. x - это скорость вперед, y - это скорость налево, z - это скорость вверх (не задействовано), yaw — это угловая скорость в рад/с (против часовой).
sit() - сесть.
stand() - встать.
give_hand() - подать лапу.
wave_hand() - помахать лапой (поднять лапу).
speak(text) - произнести текст.
get_description(prompt) - получить описание текущего кадра с камеры на голове робота-собаки. В параметр prompt помести запрос о том, что ты хочешь узнать о картинке. Для проверки наличия предмета, спроси есть ли он в кадре, и попроси ответить "да" или "нет", после можешь использовать проверку на вхождение подстроки в результате, например, "да" или "нет".
get_battery_voltage() - получить напряжение батареи в вольтах.
get_pitch() - получить угол по тангажу в радинах.
get_roll() - получить угол по крену в радианах.
get_yaw() - получить угол по рысканью в радианах.
Считай, что функции для управления роботом уже объявлены.
Максимальная скорость движения робота по x - 0.5 м/с, а по y - 0.2 м/с, по yaw - 0.85 рад/с.
Для расчета времени поворота используй формулу в коде.
Положительная скорость по yaw - это вращение против часовой, то есть налево.
Отрицательная скорость по yaw - это вращение по часовой, то есть направо.
Напряжение батареи - это общее напряжение на 6 банках. Полная зарядка - это 4.2 В на банку, а разряженная - это 3 В на банку.
Код будет выполняться при помощи функции exec.
Не используй вывод через print, используй функцию speak. Не выводи текст вместо программы, используй функцию speak. Произноси текст по-русски.
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


def get_description(prompt=None):
    time.sleep(3)
    image_processor.capture_image_ros()
    base64_image = image_processor.encode_image()
    description = image_describer.send_image_to_openai(base64_image, prompt=prompt)
    return description


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
    set_velocity(0, 0, 0, 0)
    time.sleep(2)
    set_mode(2)  # control "pose"
    while cmd_pose.angular.y < 0.5:
        cmd_pose.angular.y += 0.01
        cmd_pose_pub.publish(cmd_pose)
        rospy.sleep(0.02)


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
    time.sleep(1)
    if yaw != 0 and abs(x) < 0.1:
        x = 0.1
    cmd_vel.linear.x = x
    cmd_vel.linear.y = y
    cmd_vel.linear.z = z
    cmd_vel.angular.z = yaw # get_angular_z(yaw, x) # TODO:


def publish_cmd_vel(event):
    cmd_vel_pub.publish(cmd_vel)
    cmd_pose_pub.publish(cmd_pose)
    status_pub.publish(True)

def image_change(number):
    change_image.image_change(number)


publish_timer = rospy.Timer(rospy.Duration(1 / 50), publish_cmd_vel)


def do_command(prompt):
    program = gpt(prompt)
    program = program.replace("```python", "").replace("```", "")
    program = 'import time\n' + program
    print(program)
    g = {'set_velocity': set_velocity, 'speak': speak, 'get_battery_voltage': get_battery_voltage,
            'get_pitch': get_pitch, 'get_roll': get_roll, 'get_yaw': get_yaw, 'sit': sit, 'stand': stand,
            'give_hand': give_hand, 'wave_hand': wave_hand, "get_description": get_description, "image_change": image_change}
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
