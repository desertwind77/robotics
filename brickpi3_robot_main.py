#!/usr/bin/env python3
'''The main program for the BrickPi3 robot'''

from pathlib import Path
import argparse
import json
import logging
import os
import time

from utils import setup_logging
from utils.brickpi3_board import BrickPiEV3


CONFIG = 'config/config_brickpi3.json'


def parser_arguments() -> argparse.Namespace:
    '''Parse the command line arguments'''
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--debug", action="store_true",
                        help="Print debug information")
    subparser = parser.add_subparsers(dest='command')
    subparser.required = True

    subparser.add_parser('remote', help='Remote control robot')

    return parser.parse_args()


def load_config(filename: str, mode: str) -> dict:
    '''Load the configuration of this robot

    Args:
        filename (str): the config filename

        mode (str): operating mode

    Return:
        a dictionary containing the configuration for a specific mode
    '''
    assert os.path.exists(filename)

    config_data = None
    with open(filename, encoding="utf-8") as config_file:
        config_data = json.load(config_file)

    assert mode in config_data['robots'], f'{mode} config is not available'
    return config_data['robots'][mode]


def remote_control_robot(config: dict) -> None:
    '''Control the robot via a infrared remote control

    Args:
        config (dict): the robot configruation
    '''
    # TODO: maybe move this to the Motor class
    power = config['devices']['motors']['Left Motor']['power']

    try:
        robot = BrickPiEV3(config)

        motor_left = robot.get_device('Left Motor')
        motor_right = robot.get_device('Right Motor')

        power = 100
        while True:
            key = robot.get_infrared_remote_status(1)
            if key is not None:
                if (key.red_up == 0 and key.red_down == 0) or \
                        (key.red_up == 1 and key.red_down == 1):
                    # If both buttons are pressed or released, stop the robot.
                    power_left = 0
                elif key.red_up == 1:
                    # Forward
                    power_left = -1 * power
                else:
                    # Backward
                    power_left = power

                if (key.blue_up == 0 and key.blue_down == 0) or \
                        (key.blue_up == 1 and key.blue_down == 1):
                    # If both buttons are pressed or released, stop the robot.
                    power_right = 0
                elif key.blue_up == 1:
                    # Forward
                    power_right = -1 * power
                else:
                    # Backward
                    power_right = power

                motor_left.set_power(power_left)
                motor_right.set_power(power_right)

            time.sleep(0.01)
    except KeyboardInterrupt:
        logging.info('Good Bye!')
        pass
    finally:
        robot.reset()


def main() -> None:
    '''The main function'''
    args = parser_arguments()
    config = load_config(CONFIG, args.command)

    setup_logging()
    logging.info('Start BrickPi robot in {} mode'.format(args.command))

    if args.command == 'remote':
        remote_control_robot(config)


if __name__ == '__main__':
    main()
