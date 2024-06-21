#!/usr/bin/python3
# A class to control the basic functions of the robot

from enum import Enum
import logging
import threading
import time

from easygopigo3 import EasyGoPiGo3
from di_sensors.easy_line_follower import EasyLineFollower

from utils import HardwareException


class Direction(Enum):
   '''The Enum to represen the movement direction of the robot'''
   STOP = 0
   FORWARD = 1
   BACKWARD = 2
   LEFT = 3
   RIGHT = 4


class GoPiGoRobot:
   '''The class to represent the GoPiGo board'''
   # The line following sensor is connected to the port AD1.
   LINE_FOLLOWING_SENSOR_PORT = 'AD1'
   TILT_SERVO_PORT = 'SERVO1'
   PAN_SERVO_PORT = 'SERVO2'
   # The wheel diameter is 6.5 cm.
   WHEEL_DIAMETER_IN_CM = 6.5

   # Speed in degree per second
   SPEED_MIN_DPS = 300
   SPEED_DEFAULT_DPS = 500
   SPEED_MAX_DPS = 1000

   MIN_DISTANCE_IN_MM = 30                # Minimum distance for the distance sensor
   MAX_DISTANCE_IN_MM = 3000              # Maximum distance for the distance sensor
   FRONTAL_LOAD_IN_MM = 0

   # Default position for the pan and tilt servo motor where the camera
   # points directly forward.
   PAN_DEGREE_MIN = 0
   PAN_DEGREE_MAX = 180
   TILT_DEGREE_MIN = 0
   TILT_DEGREE_MAX = 180
   DEFAULT_PAN_POS = 95
   DEFAULT_TILT_POS = 90

   LOW_VOLTAGE_THRESHOLD = 9.0
   VOLTAGE_UPDATE_PERIOD = 5 * 60         # Update voltage every 5 min
   DISTANCE_UPDATE_PERIOD = 0.1           # This may need to change based on speed
   COLLISION_AVOIDANCE_SAFETY_FACTOR = 8

   def __init__(self) -> None:
      # Create the hardware access objects
      self.gopigo = EasyGoPiGo3(use_mutex=True)
      self.sensor_distance = self.gopigo.init_distance_sensor()
      self.sensor_line = \
         EasyLineFollower(port=GoPiGoRobot.LINE_FOLLOWING_SENSOR_PORT)
      self.servo_tilt = self.gopigo.init_servo(GoPiGoRobot.TILT_SERVO_PORT)
      self.servo_pan = self.gopigo.init_servo(GoPiGoRobot.PAN_SERVO_PORT)

      # Because the robot has a distance sensor, we expect it to support
      # multi-threads where one thread controls the movement of the robot
      # and another thread monitors the frontal distance. Under imminent
      # head-on collisin, the distance monitor thread can press the break.
      # This lock will guarantee a thread mutually exclusive access to
      # the hardware.
      self.robot_control_lock = threading.Lock()

      # Cached current statuses of the robot
      self.current_speed = None
      self.current_pan_pos = GoPiGoRobot.DEFAULT_PAN_POS
      self.current_tilt_pos = GoPiGoRobot.DEFAULT_TILT_POS
      self.current_motion = Direction.STOP
      self.imminent_collision = False

      # Stop all the motors and reset the servo motor position. This is
      # neccessary because the previous run of this program might crash
      # after it put the robot into motion or move the servo motors. We
      # want to make sure that the robot starts in a sane state right
      # after the program start.
      self.reset()

   def get_wheel_perimeter_in_cm(self) -> float:
      '''Get the wheel perimeter in cm'''
      return 2 * 3.1416 * (GoPiGoRobot.WHEEL_DIAMETER_IN_CM / 2)

   def reset(self) -> None:
      '''Stop the robot, reset the speed, and re-poisition the camera'''
      self.stop()
      self.set_speed(GoPiGoRobot.SPEED_DEFAULT_DPS)
      self.pan(GoPiGoRobot.DEFAULT_PAN_POS)
      self.tilt(GoPiGoRobot.DEFAULT_TILT_POS)

   def acquire_lock(self) -> None:
      '''Acquire the hardware lock'''
      self.robot_control_lock.acquire()

   def release_lock(self) -> None:
      '''Release the hardware lock'''
      self.robot_control_lock.release()

   def threadlock(origFunc):
      '''A decorator to handle thread lock and exception'''

      def wrapper(self, *args, **kwargs):
         result = None
         try:
            self.acquire_lock()
            result = origFunc(self, *args, **kwargs)
         except OSError as e:
            raise e
         finally:
            self.release_lock()
         return result

      return wrapper

   @threadlock
   def get_board(self) -> str:
      '''Get the board name'''
      return self.gopigo.get_board()

   @threadlock
   def get_serial_number(self) -> str:
      '''Get the serial number'''
      return self.gopigo.get_id()

   @threadlock
   def get_manufacturer(self) -> str:
      '''Get the manufacturere name'''
      return self.gopigo.get_manufacturer()

   @threadlock
   def get_hardware(self) -> str:
      '''Get the hardware version'''
      return self.gopigo.get_version_hardware()

   @threadlock
   def get_firmware(self) -> str:
      '''Get the firmware version'''
      return self.gopigo.get_version_firmware()

   @threadlock
   def get_voltage(self) -> int:
      '''Get the battery voltage'''
      return self.gopigo.get_voltage_battery()

   @threadlock
   def get_voltage5V(self) -> int:
      '''Get the 5V circuit voltage'''
      return self.gopigo.get_voltage_5v()

   @threadlock
   def set_line_type(self, lineType) -> None:
      '''Select the line color for the line following sensor'''
      assert lineType in [ 'black', 'white' ]
      self.sensor_line.set_calibration(lineType)

   @threadlock
   def get_line_position(self, weightedAvg: bool = False) -> float:
      '''Read the value from the line following sensor'''
      if weightedAvg:
         return self.sensor_line.read('weighted-avg')
      return self.sensor_line.position()

   @threadlock
   def get_distance(self) -> float:
      '''Get the frontal distance from the distance sensor'''
      if self.sensor_distance:
         # We need to check if the distance sensor is available because
         # init_distance_sensor() may fail.
         return self.sensor_distance.read_mm()
      return None

   def get_speed_unlock(self, is_km_per_hr=True) -> float:
      '''Get the current speed in degree per second or Km/h'''
      speed_in_dps = self.gopigo.get_speed()
      result = speed_in_dps
      if is_km_per_hr:
         perimeter_in_cm = self.get_wheel_perimeter_in_cm()
         speed_in_cm_per_sec = speed_in_dps * perimeter_in_cm / 360
         speed_in_km_per_hr = \
            round(speed_in_cm_per_sec * 60 * 60 / (100 * 1000), 2)
         result = speed_in_km_per_hr
      return result

   @threadlock
   def get_speed(self, is_km_per_hr=True) -> float:
      return self.get_speed_unlock(is_km_per_hr=is_km_per_hr)

   @threadlock
   def set_speed(self, newSpeed) ->  None:
      '''Set the current speed'''
      if newSpeed < GoPiGoRobot.SPEED_MIN_DPS:
         newSpeed = GoPiGoRobot.SPEED_MIN_DPS
      if newSpeed > GoPiGoRobot.SPEED_MAX_DPS:
         newSpeed = GoPiGoRobot.SPEED_MAX_DPS
      self.gopigo.set_speed(in_speed=newSpeed)
      self.current_speed = newSpeed

   @threadlock
   def set_motor_speed(self, newSpeed, is_left: bool = False) -> None:
      '''Set the speed of the left motor'''
      motor = self.gogigo.MOTOR_LEFT if is_left else self.gopigo.MOTOR_RIGHT
      self.gopigo.set_motor_dps(self.gopigo.MOTOR_LEFT, dps=newSpeed)

   def accelerate(self) -> None:
      '''Accelerate for 100 degree per second'''
      newSpeed = min(self.current_speed + 100, GoPiGoRobot.SPEED_MAX_DPS)
      if newSpeed != self.current_speed:
         self.set_speed(newSpeed)
         self.current_speed = newSpeed

   def decelerate(self) -> None:
      '''Delerate for 100 degree per second'''
      newSpeed = max(self.current_speed - 100, GoPiGoRobot.SPEED_MIN_DPS)
      if newSpeed != self.current_speed:
         self.set_speed(newSpeed)
         self.current_speed = newSpeed

   @threadlock
   def forward(self):
      if not self.imminent_collision:
         #self.current_motion != Direction.FORWARD:
         self.gopigo.forward()
         self.current_motion = Direction.FORWARD

   @threadlock
   def backward(self):
      if self.current_motion != Direction.BACKWARD:
         self.gopigo.backward()
         self.imminent_collision = False
         self.current_motion = Direction.BACKWARD

   @threadlock
   def left(self):
      if self.current_motion != Direction.LEFT:
         self.gopigo.left()
         self.imminent_collision = False
         self.current_motion = Direction.LEFT

   @threadlock
   def right(self):
      if self.current_motion != Direction.RIGHT:
         self.gopigo.right()
         self.imminent_collision = False
         self.current_motion = Direction.RIGHT

   @threadlock
   def stop(self):
      '''Stop the robot from moving'''
      if self.current_motion != Direction.STOP:
         self.gopigo.stop()
         self.imminent_collision = False
         self.current_motion = Direction.STOP

   def get_default_servo_pos(self, is_pan: bool) -> int:
      '''Get the default position of a servo motor

      Args:
         is_pan (bool): True for the pan motor and False for the tilt motor

      Return:
         the default postion of the servo motor
      '''
      return self.DEFAULT_PAN_POS if is_pan else self.DEFAULT_TILT_POS

   @threadlock
   def get_servo_pos(self, is_pan: bool) -> int:
      '''Get the current position of a servo motor

      Args:
         is_pan (bool): True for the pan motor and False for the tilt motor

      Return:
         the current postion of the servo motor
      '''
      return self.current_pan_pos if is_pan else self.current_tilt_pos

   @threadlock
   def pan(self, pos: int) -> None:
      '''Pan the camera for a certain degree'''
      pos = max(self.PAN_DEGREE_MIN, pos)
      pos = min(pos, self.PAN_DEGREE_MAX)
      self.current_pan_pos = pos
      self.servo_pan.rotate_servo(pos)

   @threadlock
   def tilt(self, pos: int) -> None:
      '''Tilt the camera for a certain degree'''
      pos = max(self.TILT_DEGREE_MIN, pos)
      pos = min(pos, self.TILT_DEGREE_MAX)
      self.current_tilt_pos = pos
      self.servo_tilt.rotate_servo(pos)

   @threadlock
   def do_voltage_check(self):
      lowVoltageMessage = \
            'Low Voltage Warning : threshold = {} volt, current =  {} volt'

      voltage = self.getVoltage()
      if voltage < GoPiGoRobot.LOW_VOLTAGE_THRESHOLD:
         logging.info(
               lowVoltageMessage.format(GoPiGoRobot.LOW_VOLTAGE_THRESHOLD, voltage))

         if self.audio_player:
            self.audio_player.speak('Low Voltage Warning')

   @threadlock
   def do_collision_avoidance(self):
      if not self.sensor_distance:
         raise HardwareException

      message = 'Collision avoidance activated : threshold = {}, ' + \
                'distance = {}, nextDistance = {}'
      threshold = GoPiGoRobot.MIN_DISTANCE_IN_MM + GoPiGoRobot.FRONTAL_LOAD_IN_MM

      perimeter_in_cm = self.get_wheel_perimeter_in_cm()
      distance_in_mm = self.sensor_distance.read_mm()
      speed_in_dps = self.get_speed_unlock(is_km_per_hr=False)
      speed_in_mm_per_sec = speed_in_dps * perimeter_in_cm * 10 / 360
      distance_to_go = speed_in_mm_per_sec * \
            GoPiGoRobot.DISTANCE_UPDATE_PERIOD * \
            GoPiGoRobot.COLLISION_AVOIDANCE_SAFETY_FACTOR

      nextDistance = round(distance_in_mm - distance_to_go)
      if nextDistance < threshold:
         logging.info(message.format(threshold, distance_in_mm, nextDistance))
         self.imminent_collision = True
         if self.current_motion == Direction.FORWARD:
            self.current_motion = Direction.STOP
            self.gopigo.stop()
      else:
         self.imminent_collision = False


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
