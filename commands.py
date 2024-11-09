import rospy

# https://github.com/voltdog/mors_base/blob/main/mors/srv/QuadrupedCmd.srv

from mors.srv import QuadrupedCmd

rospy.wait_for_service('robot_mode')
set_mode = rospy.ServiceProxy('robot_mode', QuadrupedCmd)

def sit():
    set_mode(2)
    # TODO:


def standup():
    pass
