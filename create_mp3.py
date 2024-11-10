from pathlib import Path
from openai import OpenAI
from pydub import AudioSegment
from secret import OPENAI_API_KEY


client = OpenAI(api_key=OPENAI_API_KEY, base_url="https://openai.batalov.me/v1")


def speak_ai(text: str, sync: bool = True):
    print('Speak AI: ', text)
    speech_file_path = Path(__file__).parent / "wait.mp3"

    response = client.audio.speech.create(
    model="tts-1",
    voice="alloy",
    input=text, 
    response_format="mp3",
    speed="0.8"
    )

    response.stream_to_file(speech_file_path)
    audio = AudioSegment.from_file("wait.mp3", format="mp3")
    louder_audio = audio + 20
    louder_audio.export("wait.mp3", format="mp3")


if __name__ == '__main__':
    speak_ai('Секундочку!')