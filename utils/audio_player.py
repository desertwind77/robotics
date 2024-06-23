#!/usr/bin/env python3
from collections import namedtuple
import logging
import os
import signal
import subprocess
import threading
import time

import pyttsx3

from utils import install_signal_handler, SignalException


# A tuple describing a song and its locaiton
Song = namedtuple( 'Song', 'name filename' )


class AudioPlayerBase(object):
    '''Base class for Audio Player'''
    def __init__(self) -> None:
        '''Constructor'''
        self.player_lock = threading.Lock()
        self.cur_player_process = None

    def __del__(self) -> None:
        '''Stop playing when the AudioPlayer object is destroyed.'''
        self.stop()

    def play(self, filename: str) -> None:
        '''Play an MP3 file

        Args:
            filename (str): the absolute path of the MP3 file
        '''
        # We use subprocess.Popen to suppress stdout and stderr. In addition,
        # we set shell=True because we want a non-blocking call. In effect, it
        # will spawn a new process to play the song. We can also kill the
        # process later to stop the song.
        self.stop()

        self.player_lock.acquire()
        process = subprocess.Popen('mpg321 {}'.format( filename ),
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE,
                                   shell=True, preexec_fn=os.setsid)
        self.cur_player_process = process
        self.player_lock.release()

    def stop(self) -> None:
        '''Stop the song being play if any'''
        # The idea is to use a process group so as to enable sending a signal
        # to all the process in the groups. For that, we attach a session id to
        # the parent process of the spawned/child processes, which is a shell
        # (remember shell=True). This will make it the group leader of the
        # processes. So now, when a signal is sent to the process group leader,
        # it's transmitted to all of the child processes of this group.  Ref:
        # https://stackoverflow.com/questions/4789837
        self.player_lock.acquire()

        if self.cur_player_process and ( self.cur_player_process.poll() is None ):
            # self.cur_player_process.poll() is None means
            # self.cur_player_process is still running.
            os.killpg( os.getpgid( self.cur_player_process.pid ), signal.SIGTERM )
            self.cur_player_process = None

        self.player_lock.release()

class AudioPlayer( AudioPlayerBase ):
    '''Audio player with song library'''
    PYTTSX3_VOICE_RATE = 150

    def __init__(self, audio_config: dict) -> None:
        '''Constructor'''
        super( AudioPlayer, self ).__init__()
        self.text2speech = pyttsx3.init()
        self.text2speech.setProperty( 'rate', AudioPlayer.PYTTSX3_VOICE_RATE )
        self.library = [Song(i["name"], i["path"]) for i in audio_config] \
                       if audio_config else None


    def play_library(self, index: int = 0) -> None:
        '''Play a song from the library

        Args:
            index (int): the index of the song in the library
        '''
        if not self.library:
            logging.error('Empty song library')
            return

        index = max(0, index) % len(self.library)
        song = self.library[index]
        logging.info('Playing {} ({})'.format(song.name, song.filename ))
        self.play(filename=song.filename )

    def speak(self, message: str) -> None:
        '''Say the message'''
        # This synchronous call may need to be converted to asynchronous one
        # if it has a negative impact on the performance. But for now, let's
        # live with it. It is a off-line text-to-speech. The performance hit
        # may not be that bad.
        self.text2speech.say(message)
        self.text2speech.runAndWait()
