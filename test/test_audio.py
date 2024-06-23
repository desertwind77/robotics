#!/usr/bin/env python3
import json
import time

#------------------------------ Hack ---------------------------------------#
# This is ugly but I cannot find the solution to keep this file structure yet
import sys
from pathlib import Path
path = Path(__file__)
repo_path = path.parent.parent.absolute()
sys.path.append(str(repo_path))
#---------------------------------------------------------------------------#

from utils import install_signal_handler, SignalException
from utils.audio_player import AudioPlayer


def main():
    install_signal_handler()

    with open('audio.json', 'r', encoding='utf-8') as file:
        audio_config = json.load(file)
        audio_config = audio_config['library']

    player = AudioPlayer(audio_config)

    try:
        player.speak('This is George!. George is a good little monkey')
        time.sleep(5)
        player.play_library(0)
        time.sleep(60)
    except SignalException as e:
        print( e )
    finally:
        player.stop()

if __name__ == '__main__':
    main()

