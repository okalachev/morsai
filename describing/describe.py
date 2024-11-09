import openai

class DescribeImage:
    def __init__(self, api_key):
        self.client = openai.OpenAI(api_key=api_key, base_url="https://openai.batalov.me/v1")

    def send_image_to_openai(self, base64_image):
        response = self.client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": "Что изображено на картинке?",
                        },
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{base64_image}"
                            },
                        },
                    ],
                }
            ],
        )
        return response.choices[0].message.content