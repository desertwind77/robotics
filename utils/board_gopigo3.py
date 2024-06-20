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

   def __init__( self ):
      # Create the hardware access objects
      self._gopigo = EasyGoPiGo3( use_mutex=True )
      self._distanceSensor = self._gopigo.init_distance_sensor()
      self._lineSensor = \
         EasyLineFollower( port=self.LINE_FOLLOWING_SENSOR_PORT )
      self._tilt = self._gopigo.init_servo( self.TILT_SERVO_PORT )
      self._pan = self._gopigo.init_servo( self.PAN_SERVO_PORT )

      # Because the robot has a distance sensor, we expect it to support
      # multi-threads where one thread controls the movement of the robot
      # and another thread monitors the frontal distance. Under imminent
      # head-on collisin, the distance monitor thread can press the break.
      # This lock will guarantee a thread mutually exclusive access to
      # the hardware.
      self._robotControlLock = threading.Lock()

      # Cached current statuses of the robot
      self._currentMotion = Direction.STOP
      self._currentSpeed = None
      self._currentPanPos = self.DEFAULT_PAN_POS
      self._currentTiltPos = self.DEFAULT_TILT_POS

      # Stop all the motors and reset the servo motor position. This is
      # neccessary because the previous run of this program might crash
      # after it put the robot into motion or move the servo motors. We
      # want to make sure that the robot starts in a sane state right
      # after the program start.
      self.reset()

   def reset( self ):
      '''Stop the robot, reset the speed, and re-poisition the camera'''
      self.stop()
      self.setSpeed( GoPiGoRobot.SPEED_DEFAULT_DPS )
      self.pan( self.DEFAULT_PAN_POS )
      self.tilt( self.DEFAULT_TILT_POS )

   def acquireLock( self ):
      '''Acquire the hardware lock'''
      self._robotControlLock.acquire()

   def releaseLock( self ):
      '''Release the hardware lock'''
      self._robotControlLock.release()

   def _lockAndExceptionHandler( origFunc ):
      '''A decorator to handle exception'''

      def wrapper( self, *args, **kwargs ):
         result = None
         try:
            self.acquireLock()
            result = origFunc( self, *args, **kwargs )
         except OSError as e:
            raise e
         finally:
            self.releaseLock()
         return result

      return wrapper

   @_lockAndExceptionHandler
   def getBoard( self ):
      '''Get the board name'''
      return self._gopigo.get_board()

   @_lockAndExceptionHandler
   def getSerialNo( self ):
      '''Get the serial number'''
      return self._gopigo.get_id()

   @_lockAndExceptionHandler
   def getManufacturer( self ):
      '''Get the manufacturere name'''
      return self._gopigo.get_manufacturer()

   @_lockAndExceptionHandler
   def getHardware( self ):
      '''Get the hardware version'''
      return self._gopigo.get_version_hardware()

   @_lockAndExceptionHandler
   def getFirmware( self ):
      '''Get the firmware version'''
      return self._gopigo.get_version_firmware()

   @_lockAndExceptionHandler
   def getVoltage( self ):
      '''Get the battery voltage'''
      return self._gopigo.get_voltage_battery()

   @_lockAndExceptionHandler
   def getVoltage5V( self ):
      '''Get the 5V circuit voltage'''
      return self._gopigo.get_voltage_5v()

   @_lockAndExceptionHandler
   def setLineType( self, lineType ):
      '''Select the line color for the line following sensor'''
      assert lineType in [ 'black', 'white' ]
      self._lineSensor.set_calibration( lineType )

   @_lockAndExceptionHandler
   def getLinePosition( self, weightedAvg=False ):
      '''Read the value from the line following sensor'''
      if weightedAvg:
         return self._lineSensor.read( 'weighted-avg' )
      return self._lineSensor.position()

   @_lockAndExceptionHandler
   def getDistance( self ):
      '''Get the frontal distance from the distance sensor'''
      if self._distanceSensor:
         # We need to check if the distance sensor is available because
         # init_distance_sensor() may fail.
         return self._distanceSensor.read_mm()
      return None

   @_lockAndExceptionHandler
   def getSpeed( self, isKmPerHr=True ):
      '''Get the current speed in degree per second or Km/h'''
      speedInDps = self._gopigo.get_speed()
      result = speedInDps
      if isKmPerHr:
         wheelPerimeter = self.getWheelPerimeterInCm()
         speedInCmPerSec = speedInDps * wheelPerimeter / 360
         speedInKmPerHr = \
            round( speedInCmPerSec * 60 * 60 / ( 100 * 1000 ), 2 )
         result = speedInKmPerHr
      return  result

   @_lockAndExceptionHandler
   def setSpeed( self, newSpeed ):
      '''Set the current speed'''
      if newSpeed < self.SPEED_MIN_DPS:
         newSpeed = self.SPEED_MIN_DPS
      if newSpeed > self.SPEED_MAX_DPS:
         newSpeed = self.SPEED_MAX_DPS
      self._gopigo.set_speed( in_speed=newSpeed )
      self._currentSpeed = newSpeed

   @_lockAndExceptionHandler
   def setLeftMotorSpeed( self, newSpeed ):
      '''Set the speed of the left motor'''
      self._gopigo.set_motor_dps( self._gopigo.MOTOR_LEFT, dps=newSpeed )

   @_lockAndExceptionHandler
   def setRightMotorSpeed( self, newSpeed ):
      '''Set the speed of the right motor'''
      self._gopigo.set_motor_dps( self._gopigo.MOTOR_RIGHT, dps=newSpeed )

   def accelerate( self ):
      '''Accelerate for 100 degree per second'''
      newSpeed = min( self._currentSpeed + 100, GoPiGoRobot.SPEED_MAX_DPS )
      if newSpeed != self._currentSpeed:
         self.setSpeed( newSpeed )
         self._currentSpeed = newSpeed

   def decelerate( self ):
      '''Delerate for 100 degree per second'''
      newSpeed = max( self._currentSpeed - 100, GoPiGoRobot.SPEED_MIN_DPS )
      if newSpeed != self._currentSpeed:
         self.setSpeed( newSpeed )
         self._currentSpeed = newSpeed

   @_lockAndExceptionHandler
   def getCurrentMotion( self ):
      '''Get the direction in which the robot is moving'''
      return self._currentMotion

   @_lockAndExceptionHandler
   def forward( self ):
      '''Move forward'''
      if self._currentMotion != Direction.FORWARD:
         self._gopigo.forward()
         self._currentMotion = Direction.FORWARD

   @_lockAndExceptionHandler
   def backward( self ):
      '''Move backward'''
      if self._currentMotion != Direction.BACKWARD:
         self._gopigo.backward()
         self._currentMotion = Direction.BACKWARD

   @_lockAndExceptionHandler
   def left( self ):
      '''Move left'''
      if self._currentMotion != Direction.LEFT:
         self._gopigo.left()
         self._currentMotion = Direction.LEFT

   @_lockAndExceptionHandler
   def right( self ):
      '''Move right'''
      if self._currentMotion != Direction.RIGHT:
         self._gopigo.right()
         self._currentMotion = Direction.RIGHT

   @_lockAndExceptionHandler
   def stop( self ):
      '''Stop the robot from moving'''
      if self._currentMotion != Direction.STOP:
         self._gopigo.stop()
         self._currentMotion = Direction.STOP

   @_lockAndExceptionHandler
   def pan( self, pos ):
      '''Pan the camera for a certain degree'''
      pos = max( self.PAN_DEGREE_MIN, pos )
      pos = min( pos, self.PAN_DEGREE_MAX )
      self._currentPanPos = pos
      self._pan.rotate_servo( pos )

   @_lockAndExceptionHandler
   def getPanPos( self ):
      '''Get the pan servo position in degree'''
      return self._currentPanPos

   @_lockAndExceptionHandler
   def tilt( self, pos ):
      '''Tilt the camera for a certain degree'''
      pos = max( self.TILT_DEGREE_MIN, pos )
      pos = min( pos, self.TILT_DEGREE_MAX )
      self._currentTiltPos = pos
      self._tilt.rotate_servo( pos )

   @_lockAndExceptionHandler
   def getTiltPos( self ):
      '''Get the tilt servo position in degree'''
      return self._currentTiltPos

def main():
   '''A function to test the basic functionality of the robot'''
   robot = None
   try:
      # Create a robot object
      robot = GoPiGoRobot()

      # Move forward for 5 sec
      robot.forward()
      time.sleep( 5 )

      # Move left for 5 sec
      robot.left()
      time.sleep( 5 )

      # Move right for 5 sec
      robot.right()
      time.sleep( 5 )

      # Move backward for 5 sec
      robot.backward()
      time.sleep( 5 )

      # Stop the robot
      robot.stop()

      # Pan the camera left at 15 degree
      robot.pan( 40 )
      time.sleep( 1 )
      # Pan the camera right at 125 degree
      robot.pan( 125 )
      time.sleep( 1 )

      # Tilt the camera upward at 45 degree
      robot.tilt( 45 )
      time.sleep( 1 )
      # Tilt the camera downward at 45 degree
      robot.tilt( 120 )
      time.sleep( 1 )

      # Reset the camera position
      robot.reset()

   except Exception as e:
      print( e )
   finally:
      # Always stop the motor before the program terminates.
      if robot:
         robot.reset()

if __name__ == '__main__':
   main()
