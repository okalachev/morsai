#!/usr/bin/env python3

import rospy
import atexit
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

rospy.init_node('sit')

status_pub = rospy.Publisher('/head/status', Bool, queue_size=1)
cmd_vel_pub = rospy.Publisher('/head/cmd_pose', Twist, queue_size=1)

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
    # cmd_vel.linear.t = 0.1
    cmd_vel.angular.y = 0.5

    cmd_vel_pub.publish(cmd_vel)
    status_pub.publish(True)
