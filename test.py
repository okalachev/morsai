#!/usr/bin/env python3

import rospy
import atexit
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

rospy.init_node('kvad')

status_pub = rospy.Publisher('/head/status', Bool, queue_size=1)
cmd_vel_pub = rospy.Publisher('/head/cmd_vel', Twist, queue_size=1)

def stop():
    print('stop!')
    status_pub.publish(False)
    time.sleep(1)

rospy.on_shutdown(stop)

r = rospy.Rate(10)

start = time.time()
while not rospy.is_shutdown():
    r.sleep()

    cmd_vel = Twist()
    # cmd_vel.linear.x = 1
    cmd_vel.linear.x = 0.1
    # cmd_vel.angular.z = -0.85

    cmd_vel_pub.publish(cmd_vel)
    status_pub.publish(True)
    if time.time() - start > 10:
        print('stop')
        break


while not rospy.is_shutdown():
    r.sleep()

    cmd_vel = Twist()
    # cmd_vel.linear.x = 1
    cmd_vel.linear.x = 0.0
    # cmd_vel.angular.z = -0.85

    cmd_vel_pub.publish(cmd_vel)
    status_pub.publish(True)
