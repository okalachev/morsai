#!/usr/bin/env python3

import rospy
import atexit
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from gpt_interact import prompt as ask_gpt

rospy.init_node('morsai')
rospy.loginfo('Start MORSAI')

status_pub = rospy.Publisher('/head/status', Bool, queue_size=10)
cmd_vel_pub = rospy.Publisher('/head/cmd_vel', Twist, queue_size=10)

cmd_vel = Twist()

SYSTEM_PROMPT = '''Ты управляешь роботом-собакой. Тебе нужно написать программу на Python для нее.
Можешь использовать любые функции Python, включая time.sleep и т. д.
Выдай в ответ только программу на Python, без форматирования и без ```.
Для управления используй следующие функции:
set_velocity(x, y, z) - установить скорость движения робота.
Считай, что функции для управления роботом уже объявлены.
Максимальная скорость движения робота - 1 м/с.
Код будет выполнять при помощи функции exec.
Напиши программу в соответствие с запросом пользователя:
'''

def gpt(prompt: str) -> str:
    return ask_gpt(SYSTEM_PROMPT, prompt)


def stop():
    print('stop!')
    publish_timer.shutdown()
    status_pub.publish(False)
    time.sleep(1)

rospy.on_shutdown(stop)

def set_velocity(x, y, z):
    cmd_vel.linear.x = x
    cmd_vel.linear.y = y
    cmd_vel.linear.z = z
    # cmd_vel_pub.publish(cmd_vel)
    # status_pub.publish(True)


def publish_cmd_vel(event):
    cmd_vel_pub.publish(cmd_vel)
    status_pub.publish(True)


publish_timer = rospy.Timer(rospy.Duration(1 / 10), publish_cmd_vel)


print('Input prompt')
program = gpt(input())
print(program)
g = {'set_velocity': set_velocity}
exec(program, g)


rospy.spin()
