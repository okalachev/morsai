import cv2
import base64
import openai

from secret import OPENAI_API_KEY
from describe import DescribeImage
from image_processing import ImageProcessing

def main():
    image_processor = ImageProcessing()
    image_processor.capture_image()
    base64_image = image_processor.encode_image()

    image_describer = DescribeImage(OPENAI_API_KEY)
    description = image_describer.send_image_to_openai(base64_image)
    
    print(description)


if __name__ == "__main__":
    main()