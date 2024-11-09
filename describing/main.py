import cv2
import base64
import openai

from describe import DescribeImage
from image_processing import ImageProcessing

def main():
    image_processor = ImageProcessing()
    image_processor.capture_image()
    base64_image = image_processor.encode_image()

    api_key = "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"
    image_describer = DescribeImage(api_key)
    description = image_describer.send_image_to_openai(base64_image)
    
    print(description)


if __name__ == "__main__":
    main()