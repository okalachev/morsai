import rospy
from openai import OpenAI

from audio_common_msgs.msg import AudioData
from secret import OPENAI_API_KEY
from io import BytesIO
from std_msgs.msg import String, Bool, Empty
import sys
from threading import Timer, Lock
import time
from copy import copy

from speak import speak_start_thinking

import wave

client = OpenAI(api_key=OPENAI_API_KEY, base_url="https://openai.batalov.me/v1")
audio_sub = None
timer = None
on_audio_callback_lock = Lock()


rospy.init_node("openai_recognize")

buffer_lock = Lock()
buffer = b''
buffer_update_time = 0

class BytesIOWithClose(BytesIO):
    def flush(self):
        pass


def on_audio(audio: AudioData):
    global buffer_update_time, buffer, timer
    print("add_buffer_no_lock")
    with buffer_lock:
        print("add_buffer")
        print(type(audio.data))
        buffer_update_time = time.time()
        buffer += audio.data
        
        timer.cancel()
        timer = Timer(3, stop)
        timer.start()
    # global timer
    # with on_audio_callback_lock:
    #     print("get audio")
    #     if timer:
    #         timer.cancel()
    #         timer = None
    #     with BytesIOWithClose() as bio:
    #         out_f = wave.Wave_write(bio)
    #         out_f.setnchannels(1)
    #         out_f.setsampwidth(2) # number of bytes
    #         out_f.setframerate(16000)
    #         out_f.writeframesraw(audio.data)
    #         bio.seek(0)
    #         bio.name = "speech.wav"

    #         reco = client.audio.transcriptions.create(
    #             model="whisper-1",
    #             file=bio,
    #         )
    #         print(reco)
    #     timer = Timer(2, stop)
    #     timer.start()


def timer_callback(event):
    global timer, buffer
    do_play = False
    with buffer_lock:
        if time.time() - buffer_update_time > 3 and buffer:
            do_play = True
            buffer_copy = buffer
            buffer = b''
            timer = Timer(2, stop)
            timer.start()
    
    if not do_play:
        return
    
    print("get audio")
    if timer:
        timer.cancel()
        timer = None
    speak_start_thinking()
    with BytesIOWithClose() as bio:
        out_f = wave.Wave_write(bio)
        out_f.setnchannels(1)
        out_f.setsampwidth(2) # number of bytes
        out_f.setframerate(16000)
        out_f.writeframesraw(buffer_copy)
        bio.seek(0)
        bio.name = "speech.wav"

        reco = client.audio.transcriptions.create(
            model="whisper-1",
            file=bio,
            response_format="text",
            language="ru"
        )
        print(reco)
        voice_command_pub.publish(reco)



rospy.Timer(rospy.Duration(0.2), timer_callback)
    

def stop():
    global timer
    print("STOP!!!")
    if timer:
        timer.cancel()
        timer = None
    audio_pub.publish(False)
    stop_signal_pub.publish(Empty())


def on_asr_topic(flag: Bool):
    print("on_asr_topic", flag)
    global audio_sub, timer
    if not flag.data and audio_sub is not None:
        print("asr stop")
        audio_sub.unregister()
        audio_sub = None
    if flag.data and audio_sub is None:
        print("asr start")
        time.sleep(1)
        audio_sub = rospy.Subscriber("/head/respeaker/speech_audio", AudioData, on_audio)
        timer = Timer(20, stop)
        timer.start()






audio_pub = rospy.Publisher("/head/voice_recognizer/start_asr", Bool, queue_size=1)
stop_signal_pub = rospy.Publisher('/head/voice_recognizer/grammar_not_found', Empty, queue_size=1)
rospy.Subscriber("/head/voice_recognizer/start_asr", Bool, on_asr_topic)

voice_command_pub = rospy.Publisher("/voice_command", String, queue_size=1)

time.sleep(1)

stop()

print("ready")

rospy.spin()
