#!/usr/bin/env python3
'''A basic line following robot'''

import logging
import traceback

from utils import install_signal_handler, SignalException
from utils.gopigo3_board import GoPiGoRobot


class LineFollowingRobot(GoPiGoRobot):
    ''' This is a very crude line following robot. It just use brute force
    appraoch.'''

    def do_line_following(self):
        position = self.get_line_position()
        logging.debug('Line position = {}'.format(position))

        if position == 'center':
            self.forward()
        elif position == 'left':
            self.left()
        elif position == 'right':
            self.right()
        else:
            # Do nothing when the sensor read 'white'
            pass

    def run(self, debug=False):
        install_signal_handler()

        running = True
        while running:
            try:
                self.do_line_following()
            except (OSError, SignalException) as e:
                # Exit the main loop in the next iteration
                running = False
                # Stop the robot
                self.stop()

                # Line sensor read error will result in OSError
                print( e )
                if debug:
                    traceback.print_exc()
