import cv2
import base64
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

class ImageProcessing:
    def __init__(self, filename="captured_image.jpg"):
        self.filename = filename
        self.bridge = CvBridge()
        if not rospy.core.is_initialized():
            rospy.init_node('image_processor', anonymous=True)

    def capture_image_ros(self):
        image_msg = rospy.wait_for_message("/cv_camera/image_raw", Image)
        
        try:
            frame = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            cv2.imwrite(self.filename, frame)
            print(f"Image saved as {self.filename}")
        except CvBridgeError as e:
            print(f"Failed to convert image: {e}")

    def encode_image(self):
        with open(self.filename, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode('utf-8')
