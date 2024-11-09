import cv2
import base64

class ImageProcessing:
    def __init__(self, filename="captured_image.jpg"):
        self.filename = filename

    def capture_image(self):
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        if ret:
            cv2.imwrite(self.filename, frame)
        cap.release()

    def encode_image(self):
        with open(self.filename, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode('utf-8')