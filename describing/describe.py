import openai

class DescribeImage:
    def __init__(self, api_key):
        self.client = openai.OpenAI(api_key=api_key, base_url="https://openai.batalov.me/v1")

    def send_image_to_openai(self, base64_image, prompt=None):
        response = self.client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": 'Тебе дан кадр с камеры робота. {}'.format(prompt or "Что ты видишь? Начни со слов \"я вижу\""),
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
        ret = response.choices[0].message.content
        print(ret)
        return ret