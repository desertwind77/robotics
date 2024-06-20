#!/usr/bin/python3
# A class to control the basic functions of the robot

from enum import Enum
import threading
import time

from easygopigo3 import EasyGoPiGo3
from di_sensors.easy_line_follower import EasyLineFollower

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

   # Default position for the pan and tilt servo motor where the camera
   # points directly forward.
   PAN_DEGREE_MIN = 0
   PAN_DEGREE_MAX = 180
   TILT_DEGREE_MIN = 0
   TILT_DEGREE_MAX = 180
   DEFAULT_PAN_POS = 95
   DEFAULT_TILT_POS = 90

   def __init__(self):
      # Create the hardware access objects
      self.gopigo = EasyGoPiGo3(use_mutex=True)
      self.distance_sensor = self.gopigo.init_distance_sensor()
      self.sensor_line = \
         EasyLineFollower(port=self.LINE_FOLLOWING_SENSOR_PORT)
      self.servo_tilt = self.gopigo.init_servo(self.TILT_SERVO_PORT)
      self.servo_pan = self.gopigo.init_servo(self.PAN_SERVO_PORT)

      # Because the robot has a distance sensor, we expect it to support
      # multi-threads where one thread controls the movement of the robot
      # and another thread monitors the frontal distance. Under imminent
      # head-on collisin, the distance monitor thread can press the break.
      # This lock will guarantee a thread mutually exclusive access to
      # the hardware.
      self._robotControlLock = threading.Lock()

      # Cached current statuses of the robot
      self.current_motion = Direction.STOP
      self.current_speed = None
      self.current_pan_pos = self.DEFAULT_PAN_POS
      self.current_tilt_pos = self.DEFAULT_TILT_POS

      # Stop all the motors and reset the servo motor position. This is
      # neccessary because the previous run of this program might crash
      # after it put the robot into motion or move the servo motors. We
      # want to make sure that the robot starts in a sane state right
      # after the program start.
      self.reset()

   def reset(self):
      '''Stop the robot, reset the speed, and re-poisition the camera'''
      self.stop()
      self.set_speed(GoPiGoRobot.SPEED_DEFAULT_DPS)
      self.pan(self.DEFAULT_PAN_POS)
      self.tilt(self.DEFAULT_TILT_POS)

   def acquire_lock(self):
      '''Acquire the hardware lock'''
      self._robotControlLock.acquire()

   def release_lock(self):
      '''Release the hardware lock'''
      self._robotControlLock.release()

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
   def get_board(self):
      '''Get the board name'''
      return self.gopigo.get_board()

   @threadlock
   def get_serial_number(self):
      '''Get the serial number'''
      return self.gopigo.get_id()

   @threadlock
   def get_manufacturer(self):
      '''Get the manufacturere name'''
      return self.gopigo.get_manufacturer()

   @threadlock
   def get_hardware(self):
      '''Get the hardware version'''
      return self.gopigo.get_version_hardware()

   @threadlock
   def get_firmware(self):
      '''Get the firmware version'''
      return self.gopigo.get_version_firmware()

   @threadlock
   def get_voltage(self):
      '''Get the battery voltage'''
      return self.gopigo.get_voltage_battery()

   @threadlock
   def get_voltage5V(self):
      '''Get the 5V circuit voltage'''
      return self.gopigo.get_voltage_5v()

   @threadlock
   def set_line_type(self, lineType):
      '''Select the line color for the line following sensor'''
      assert lineType in [ 'black', 'white' ]
      self.sensor_line.set_calibration(lineType)

   @threadlock
   def get_line_position(self, weightedAvg=False):
      '''Read the value from the line following sensor'''
      if weightedAvg:
         return self.sensor_line.read('weighted-avg')
      return self.sensor_line.position()

   @threadlock
   def get_distance(self):
      '''Get the frontal distance from the distance sensor'''
      if self.distance_sensor:
         # We need to check if the distance sensor is available because
         # init_distance_sensor() may fail.
         return self.distance_sensor.read_mm()
      return None

   @threadlock
   def get_speed(self, isKmPerHr=True):
      '''Get the current speed in degree per second or Km/h'''
      speedInDps = self.gopigo.get_speed()
      result = speedInDps
      if isKmPerHr:
         wheelPerimeter = self.getWheelPerimeterInCm()
         speedInCmPerSec = speedInDps * wheelPerimeter / 360
         speedInKmPerHr = \
            round(speedInCmPerSec * 60 * 60 / (100 * 1000), 2)
         result = speedInKmPerHr
      return  result

   @threadlock
   def set_speed(self, newSpeed):
      '''Set the current speed'''
      if newSpeed < self.SPEED_MIN_DPS:
         newSpeed = self.SPEED_MIN_DPS
      if newSpeed > self.SPEED_MAX_DPS:
         newSpeed = self.SPEED_MAX_DPS
      self.gopigo.set_speed(in_speed=newSpeed)
      self.current_speed = newSpeed

   @threadlock
   def set_left_motor_speed(self, newSpeed):
      '''Set the speed of the left motor'''
      self.gopigo.set_motor_dps(self.gopigo.MOTOR_LEFT, dps=newSpeed)

   @threadlock
   def set_right_motor_speed(self, newSpeed):
      '''Set the speed of the right motor'''
      self.gopigo.set_motor_dps(self.gopigo.MOTOR_RIGHT, dps=newSpeed)

   def accelerate(self):
      '''Accelerate for 100 degree per second'''
      newSpeed = min(self.current_speed + 100, GoPiGoRobot.SPEED_MAX_DPS)
      if newSpeed != self.current_speed:
         self.set_speed(newSpeed)
         self.current_speed = newSpeed

   def decelerate(self):
      '''Delerate for 100 degree per second'''
      newSpeed = max(self.current_speed - 100, GoPiGoRobot.SPEED_MIN_DPS)
      if newSpeed != self.current_speed:
         self.set_speed(newSpeed)
         self.current_speed = newSpeed

   @threadlock
   def get_current_motion(self):
      '''Get the direction in which the robot is moving'''
      return self.current_motion

   @threadlock
   def forward(self):
      '''Move forward'''
      if self.current_motion != Direction.FORWARD:
         self.gopigo.forward()
         self.current_motion = Direction.FORWARD

   @threadlock
   def backward(self):
      '''Move backward'''
      if self.current_motion != Direction.BACKWARD:
         self.gopigo.backward()
         self.current_motion = Direction.BACKWARD

   @threadlock
   def left(self):
      '''Move left'''
      if self.current_motion != Direction.LEFT:
         self.gopigo.left()
         self.current_motion = Direction.LEFT

   @threadlock
   def right(self):
      '''Move right'''
      if self.current_motion != Direction.RIGHT:
         self.gopigo.right()
         self.current_motion = Direction.RIGHT

   @threadlock
   def stop(self):
      '''Stop the robot from moving'''
      if self.current_motion != Direction.STOP:
         self.gopigo.stop()
         self.current_motion = Direction.STOP

   def get_default_servo_pos(self, is_pan: bool) -> bool:
      '''Get the default position of a servo motor

      Args:
         is_pan (bool): True for the pan motor and False for the tilt motor

      Return:
         the default postion of the servo motor
      '''
      return self.DEFAULT_PAN_POS if is_pan else self.DEFAULT_TILT_POS

   @threadlock
   def get_servo_pos(self, is_pan: bool) -> bool:
      '''Get the current position of a servo motor

      Args:
         is_pan (bool): True for the pan motor and False for the tilt motor

      Return:
         the current postion of the servo motor
      '''
      return self.current_pan_pos if is_pan else self.current_tilt_pos

   @threadlock
   def pan(self, pos):
      '''Pan the camera for a certain degree'''
      pos = max(self.PAN_DEGREE_MIN, pos)
      pos = min(pos, self.PAN_DEGREE_MAX)
      self.current_pan_pos = pos
      self.servo_pan.rotate_servo(pos)

   @threadlock
   def tilt(self, pos):
      '''Tilt the camera for a certain degree'''
      pos = max(self.TILT_DEGREE_MIN, pos)
      pos = min(pos, self.TILT_DEGREE_MAX)
      self.current_tilt_pos = pos
      self.servo_tilt.rotate_servo(pos)


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
