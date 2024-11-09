import subprocess
from pathlib import Path
from openai import OpenAI
from pydub import AudioSegment
import rospy
from speakers_controller.srv import playSound, playSoundRequest
from secret import OPENAI_API_KEY


client = OpenAI(api_key=OPENAI_API_KEY, base_url="https://openai.batalov.me/v1")
rospy.wait_for_service('playSound')
service_playSound = rospy.ServiceProxy('playSound', playSound, persistent=True)


def speak_ai(text: str):
    print('Speak AI: ', text)
    speech_file_path = Path(__file__).parent / "speech.mp3"

    response = client.audio.speech.create(
    model="tts-1",
    voice="shimmer",
    input=text, 
    response_format="mp3",
    speed="0.8"
    )

    response.stream_to_file(speech_file_path)
    audio = AudioSegment.from_file("/home/pi/morsai/speech.mp3", format="mp3")
    louder_audio = audio + 20
    louder_audio.export("/home/pi/morsai/speech_louder.mp3", format="mp3")

    print('Sound generated, play')
    request = playSoundRequest()
    request.FileName = "/home/pi/morsai/speech_louder.mp3"
    request.IsBreakable = 0
    service_playSound(request)


def speak(text: str):
    rate = 100
    pitch = 30
    voice = 'ru'
    # subprocess.call(['espeak', '-v', voice, text])
    subprocess.call(['espeak', '-s', str(rate), '-p', str(pitch), '-v', voice, text])


if __name__ == '__main__':
    rospy.init_node("speaker")
    speak_ai('Раз, два, три, четыре, пять, я хочу выйти погулять')
    print('done')
    # speak('Гаф гаф!')
