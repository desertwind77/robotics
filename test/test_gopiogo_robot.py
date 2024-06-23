#!/usr/bin/env python3
'''Tes the basic functinality of GoPiGo robot'''
import time

#------------------------------ Hack ---------------------------------------#
# This is ugly but I cannot find the solution to keep this file structure yet
import sys
from pathlib import Path
path = Path(__file__)
repo_path = path.parent.parent.absolute()
sys.path.append(str(repo_path))
#---------------------------------------------------------------------------#

from utils.board_gopigo3 import GoPiGoRobot


def main():
   '''A function to test the basic functionality of the robot'''
   robot = None
   try:
      # Create a robot object
      robot = GoPiGoRobot()

      # Move forward for 5 sec
      robot.forward()
      time.sleep(5)

      # Move left for 5 sec
      robot.left()
      time.sleep(5)

      # Move right for 5 sec
      robot.right()
      time.sleep(5)

      # Move backward for 5 sec
      robot.backward()
      time.sleep(5)

      # Stop the robot
      robot.stop()

      # Pan the camera left at 15 degree
      robot.pan(40)
      time.sleep(1)
      # Pan the camera right at 125 degree
      robot.pan(125)
      time.sleep(1)

      # Tilt the camera upward at 45 degree
      robot.tilt(45)
      time.sleep(1)
      # Tilt the camera downward at 45 degree
      robot.tilt(120)
      time.sleep(1)

      # Reset the camera position
      robot.reset()

   except Exception as e:
      print(e)
   finally:
      # Always stop the motor before the program terminates.
      if robot:
         robot.reset()

if __name__ == '__main__':
   main()

