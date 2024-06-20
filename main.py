#!/usr/bin/env python3
'''
The program to control the robot
1) Receive input from a USB gamepad
2) Receive input from a USB keyboard
3) Receive input from a web interface over the network
'''

import argparse
import logging
import time
import traceback

from utils import install_signal_handler, setup_logging
from utils.board_gopigo3 import GoPiGoRobot
from utils.input_monitor import InputMonitor


def parser_arguments() -> argparse.Namespace:
   '''Parse the command line argument

   Return:
      argparse.Namespace: parsed command line arguments
   '''
   parser = argparse.ArgumentParser(description="Robot")
   parser.add_argument("-d", "--debug", action="store_true",
                       help="Print debug information")
   return parser.parse_args()


def main() -> None:
   '''The main program'''
   args = parser_arguments()

   # Handle SIGINT and SIGTERM
   install_signal_handler()
   setup_logging(verbose=args.debug)

   # Our Robot hardware
   robot = GoPiGoRobot()
   # Create the InputMonitor thread to watch for the user's input
   # from keyboard, gamepad, or network
   input_monitor_thread = InputMonitor(robot, None)
   # Start the InputMonitor thread
   input_monitor_thread.start()
   input_monitor_thread.help()

   # TODO:
   # 1) Exit the main thread gracefully when an exception
   #    occurs in one of the threads.
   try:
      while True:
         # The main thread do nothing.
         time.sleep(0.5)
   except Exception as e:
      print(e)
      if args.debug:
         # Print traceback
         traceback.print_exc()
   finally:
      # Shutdown the InputMonitor thread gracefully
      input_monitor_thread.shutdown()
      # Call thread join to clean up the thread resources
      input_monitor_thread.join()
      logging.info('Good Bye!')


if __name__ == '__main__':
   main()
