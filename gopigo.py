#!/usr/bin/env python3
'''
The main program for the GoPiGo3 robot
1) Receive input from a USB gamepad
2) Receive input from a USB keyboard
3) Receive input from a web interface over the network
4) Collision avoidance

Caveats:
- Need to press Ctrl-C multiple times to terminate the program
'''

import argparse
import json
import os
from utils import setup_logging
from utils.gopigo3_remote import RemoteRobot


def parser_arguments() -> argparse.Namespace:
    '''Parse the command line argument

    Return:
        argparse.Namespace: parsed command line arguments
    '''
    parser = argparse.ArgumentParser(description="Robot")
    parser.add_argument("-a", "--avoid", action="store_true",
                        help="Enable collision avoidance")
    parser.add_argument("-A", "--audio", action="store",
                        help="Audio configuration file"),
    parser.add_argument("-d", "--debug", action="store_true",
                        help="Print debug information")
    return parser.parse_args()


def main() -> None:
    args = parser_arguments()
    setup_logging(verbose=args.debug)

    audio_config = None
    if args.audio:
        assert os.path.exists(args.audio)
        with open(args.audio, 'r', encoding='utf-8') as file:
            audio_config = json.load(file)
            audio_config = audio_config['library']

    robot = RemoteRobot(audio_config)
    robot.run(args.avoid, args.debug)


if __name__ == '__main__':
   main()
