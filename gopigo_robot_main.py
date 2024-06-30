#!/usr/bin/env python3
'''
The main program for the GoPiGo3 robot with the following functionalities
1) Remote Control Robot
    - Receive input from a USB gamepad
    - Receive input from a USB keyboard
    - Receive input from a web interface over the network
    - Collision avoidance
2) Line Following Robot
3) PID Line Following Robot
4) Ball Tracking Robot

Known caveats:
- Need to press Ctrl-C multiple times to terminate the program
'''

import argparse
import json
import logging
import os
from utils import setup_logging
from utils.gopigo3_board import GoPiGoRobot
from utils.gopigo3_remote_robot import RemoteRobot
from utils.gopigo3_line_robot import LineFollowingRobot
from utils.gopigo3_pid_robot import PIDLineFollowingRobot
from utils.gopigo3_ball_robot import BallTrackingRobot


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

    # Reset the robot
    reset_parser = subparser.add_parser('reset', help='Reset the robot')

    # Robot with remote control, web control and collision avoidance
    remote_parser = subparser.add_parser('remote', help='Remote control robot')
    remote_parser.add_argument("-a", "--avoid", action="store_true",
                               help="Enable collision avoidance")
    remote_parser.add_argument("-A", "--audio", action="store",
                               help="Audio configuration file"),

    # Basic line follower
    line_parser = subparser.add_parser('line', help='Basic line following robot')

    # PID line follower
    pid_parser = subparser.add_parser('pid', help='Basic line following robot')

    # Ball tracking robot
    ball_parser = subparser.add_parser('ball', help='Ball tracking robot')
    ball_parser.add_argument("-c", "--color", action="store",
                             default="Green", choices=["Green", "Blue"],
                             help="Choose the ball color")
    ball_parser.add_argument("--convert", action="store",
                             help='Convert "red green blue" to lower and upper HSV bounds')
    ball_parser.add_argument("--display", action="store_true",
                             help="Display the video frame on the screen")
    ball_parser.add_argument("-m", "--max", action="store",
                             help="Maximum number of images to save")
    ball_parser.add_argument("-s", "--save", action="store",
                             help="Interval in seconds to save the captured images")

    return parser.parse_args()


def main() -> None:
    args = parser_arguments()
    setup_logging(verbose=args.debug)

    if args.command == 'reset':
        robot = GoPiGoRobot()
        robot.reset()
    elif args.command == 'remote':
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
        robot = PIDLineFollowingRobot()
        robot.run(args.debug)
    elif args.command == 'ball':
        robot = BallTrackingRobot(args.color)
        if args.convert:
            rgb = args.convert.split()
            lower_bound, upper_bound = \
                    robot.convert_rgb_to_hsv(rgb[0], rgb[1], rgb[2])
            logging.info('Lower bound = {}'.format(lower_bound))
            logging.info('Upper bound = {}'.format(upper_bound))
        else:
            robot.run(display_on_screen=args.display,
                      image_save_interval=int(args.save),
                      max_save_image_num=int(args.max),
                      debug=args.debug)

if __name__ == '__main__':
   main()
