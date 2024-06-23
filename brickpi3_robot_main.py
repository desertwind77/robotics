#!/usr/bin/env python3
'''The main program for the BrickPi3 robot'''

from pathlib import Path
import argparse
import json
import os
import time

from utils.brickpi3_board import BrickPiEV3


def parser_arguments() -> argparse.Namespace:
    '''Parse the command line arguments'''
    parser = argparse.ArgumentParser()
    parser.add_argument( 'mode', action='store', help='Robot mode' )
    return parser.parse_args()


def load_config(mode: str) -> dict:
    '''Load the configuration of this robot'''
    config_filename = str( Path( __file__ ).stem ) + '.json'
    script_path = os.path.realpath( os.path.dirname( __file__ ) )
    config_filename = os.path.join( script_path, config_filename )
    assert os.path.exists( config_filename )

    config_data = None
    with open( config_filename, encoding="utf-8" ) as config_file:
        config_data = json.load( config_file )

    assert mode in config_data[ 'robots' ], f'{mode} config is not available'
    return config_data[ 'robots' ][ mode ]


def remote_control_robot(config: str) -> None:
    # TODO: maybe move this to the Motor class
    power = config[ 'devices' ][ 'motors' ][ 'Left Motor' ][ 'power' ]

    try:
        robot = BrickPiEV3( config )

        motor_left = robot.get_device( 'Left Motor' )
        motor_right = robot.get_device( 'Right Motor' )

        power = 100
        while True:
            key = robot.get_infrared_remote_status( 1 )
            if key is not None:
                if ( key.red_up == 0 and key.red_down == 0 ) or \
                        ( key.red_up == 1 and key.red_down == 1 ):
                    # If both buttons are pressed or released, stop the robot.
                    power_left = 0
                elif key.red_up == 1:
                    # Forward
                    power_left = -1 * power
                else:
                    # Backward
                    power_left = power

                if ( key.blue_up == 0 and key.blue_down == 0 ) or \
                        ( key.blue_up == 1 and key.blue_down == 1 ):
                    # If both buttons are pressed or released, stop the robot.
                    power_right = 0
                elif key.blue_up == 1:
                    # Forward
                    power_right = -1 * power
                else:
                    # Backward
                    power_right = power

                motor_left.set_power( power_left )
                motor_right.set_power( power_right )

            time.sleep( 0.01 )
    except KeyboardInterrupt:
        pass
    finally:
        robot.reset()


# The dispatch table
dispatcher = {
    'remote' : remote_control_robot,
}


def dispatch(mode: str) -> None:
    '''Call the function for the robot mode'''
    assert mode in dispatcher, f'{mode} is not available in the dispatcher'
    config = load_config( mode )
    dispatcher[mode](config)


def main() -> None:
    '''The main function'''
    args = parser_arguments()
    dispatch( args.mode )


if __name__ == '__main__':
    main()
