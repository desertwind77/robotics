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
from utils import setup_logging
from utils.board_gopigo3 import RemoteRobot


def parser_arguments() -> argparse.Namespace:
    '''Parse the command line argument

    Return:
        argparse.Namespace: parsed command line arguments
    '''
    parser = argparse.ArgumentParser(description="Robot")
    parser.add_argument("-a", "--avoid", action="store_true",
                        help="Enable collision avoidance")
    parser.add_argument("-d", "--debug", action="store_true",
                        help="Print debug information")
    return parser.parse_args()


def main() -> None:
    args = parser_arguments()
    setup_logging(verbose=args.debug)
    robot = RemoteRobot()
    robot.run(args.avoid, args.debug)


if __name__ == '__main__':
   main()
