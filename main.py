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


# How often the main loop should run
MAIN_LOOP_PERIOD = 0.02


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
   '''The main program'''
   args = parser_arguments()

   # Handle SIGINT and SIGTERM
   install_signal_handler()
   setup_logging(verbose=args.debug)

   # Our audio player
   audio_player = None
   # Our Robot hardware
   robot = GoPiGoRobot()
   # Create the InputMonitor thread to watch for the user's input
   # from keyboard, gamepad, or network
   input_monitor_thread = InputMonitor(robot, None)
   # Start the InputMonitor thread
   input_monitor_thread.start()
   input_monitor_thread.help()

   try:
      last_voltage_time = last_distance_time = time.time()
      while input_monitor_thread.is_alive():
         cur_time = time.time()

         time_voltage = cur_time - last_voltage_time
         if time_voltage > robot.VOLTAGE_UPDATE_PERIOD:
            low_battery = robot.do_voltage_check()
            if audio_player and low_battery:
               audio_player.speak('Low Voltage Warning')
            last_voltage_time = cur_time

         time_distance = cur_time - last_distance_time
         if args.avoid and \
               time_distance > robot.DISTANCE_UPDATE_PERIOD:
            robot.do_collision_avoidance()
            last_distance_time = cur_time

         # The main thread do nothing.
         time.sleep(MAIN_LOOP_PERIOD)
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
