import rospy # Библиотека для работы с ROS
import os

from display_controller.srv import displayControllerPlay

service_display_player = rospy.ServiceProxy('displayControllerPlay', displayControllerPlay)
pics = {1: 'attention',
        2: 'default',
        3: 'happy',
        4: 'happy', 
        5: 'laying',
        6: 'sad',
        7: 'sleeping',
        8: 'thinking'}

def image_change(pic_number: int):
    script_path = os.path.dirname(os.path.abspath(__file__))
    service_display_player(script_path+'/faces/{}.png'.format(pics[pic_number]))



if __name__ == '__main__':
    rospy.init_node("image_changer")
    