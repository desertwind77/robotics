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
from utils.gopigo3_remote_robot import RemoteRobot
from utils.gopigo3_line_robot import LineFollowingRobot
from utils.gopigo3_pid_robot import PIDRobot


def parser_arguments() -> argparse.Namespace:
    '''Parse the command line argument

    Return:
        argparse.Namespace: parsed command line arguments
    '''
    parser = argparse.ArgumentParser(description="Robot")
    parser.add_argument("-d", "--debug", action="store_true",
                        help="Print debug information")
    subparser = parser.add_subparsers(dest='command')
    subparser.required = True

    remote_parser = subparser.add_parser('remote', help='Remote control robot')
    remote_parser.add_argument("-a", "--avoid", action="store_true",
                               help="Enable collision avoidance")
    remote_parser.add_argument("-A", "--audio", action="store",
                               help="Audio configuration file"),

    line_parser = subparser.add_parser('line', help='Basic line following robot')

    pid_parser = subparser.add_parser('pid', help='Basic line following robot')

    ball_parser = subparser.add_parser('ball', help='Ball tracking robot')

    return parser.parse_args()


def main() -> None:
    args = parser_arguments()
    setup_logging(verbose=args.debug)

    if args.command == 'remote':
        audio_config = None
        if args.audio:
            assert os.path.exists(args.audio)
            with open(args.audio, 'r', encoding='utf-8') as file:
                audio_config = json.load(file)
                audio_config = audio_config['library']

        robot = RemoteRobot(audio_config)
        robot.run(args.avoid, args.debug)
    elif args.command == 'line':
        robot = LineFollowingRobot()
        robot.run(args.debug)
    elif args.command == 'pid':
        robot = PIDRobot()
        robot.run(args.debug)
    elif args.command == 'ball':
        pass


if __name__ == '__main__':
   main()
