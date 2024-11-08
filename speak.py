import subprocess


def speak(text: str):
    rate = 100
    pitch = 30
    voice = 'ru'
    # subprocess.call(['espeak', '-v', voice, text])
    subprocess.call(['espeak', '-s', str(rate), '-p', str(pitch), '-v', voice, text])


if __name__ == '__main__':
    # speak('Всем привет!')
    speak('Гаф гаф!')

