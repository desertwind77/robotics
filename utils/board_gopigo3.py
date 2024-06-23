#!/usr/bin/python3
# A class to control the basic functions of the robot

from enum import Enum
import logging
import threading
import time
import traceback

from easygopigo3 import EasyGoPiGo3
from di_sensors.easy_line_follower import EasyLineFollower

from utils import install_signal_handler, SignalException
from utils.audio_player import AudioPlayer
from utils.input_monitor import InputMonitor


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

    # Minimum distance that the distance sensor registers
    MIN_DISTANCE_IN_MM = 30
    # Maximum distance that the distance sensor registers
    MAX_DISTANCE_IN_MM = 3000
    # The distance we want to have in front of the robot for safety
    FRONTAL_SAFE_DISTANCE_IN_MM = 0

    # Default position for the pan and tilt servo motor where the camera
    # points directly forward.
    DEFAULT_PAN_POS = 95
    DEFAULT_TILT_POS = 90
    PAN_DEGREE_MIN = 0
    PAN_DEGREE_MAX = 180
    TILT_DEGREE_MIN = 0
    TILT_DEGREE_MAX = 180

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
        self.current_speed = 0
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
        '''Acquire the hardware lock to provide mutual exclusive
        access to the hardware'''
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
        '''Get the battery voltage from the main circuit'''
        return self.gopigo.get_voltage_battery()

    @threadlock
    def get_voltage5V(self) -> int:
        '''Get the battery voltage from the 5V circuit'''
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
        '''The thread-safe version of get_speed_unlock'''
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
        motor = self.gogigo.MOTOR_LEFT if is_left else self.gopigo.MOTOR_RIGHTi
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


class ThreadOperation(Enum):
    START = 0
    STOP = 1
    JOIN = 2


class RemoteRobot(GoPiGoRobot):
    # Alarm if voltage is below 9 volt
    LOW_VOLTAGE_THRESHOLD = 9.0
    # Safety factor
    COLLISION_AVOIDANCE_SAFETY_FACTOR = 8
    # Update voltage level every 5 min
    VOLTAGE_UPDATE_PERIOD = 5 * 60
    # This may need to change based on speed
    DISTANCE_UPDATE_PERIOD = 0.1
    # The main loop interval
    MAIN_LOOP_PERIOD = 0.02

    def __init__(self, audio_config):
        super(RemoteRobot, self).__init__()
        self.audio_player = AudioPlayer(audio_config)
        self.threads = []

    def initialize(self):
        # From my experience, multiple Ctrl-C may be required to terminate
        # the program. Seemingly, the SignalException that is raised in
        # response to Ctrl-C get lost when the program is reading the hardware.
        install_signal_handler()
        # Create the InputMonitor thread to watch for the user's input
        # from keyboard, gamepad, or network
        self.threads.append(InputMonitor(self, self.audio_player))

    def manager_worker_threads(self, operation):
        for thread in self.threads:
            if operation == ThreadOperation.START:
                thread.start()
            elif operation == ThreadOperation.STOP:
                thread.shutdown()
            elif operation == ThreadOperation.JOIN:
                thread.join()

    def all_threads_alive(self):
        return all([i.is_alive() for i in self.threads])

    def is_low_voltage(self) -> bool:
        '''Check if the voltage is low e.g. low battery'''
        lowVoltageMessage = \
                'Low Voltage Warning : threshold = {} volt, current =  {} volt'

        voltage = self.get_voltage()
        if voltage < GoPiGoRobot.LOW_VOLTAGE_THRESHOLD:
            logging.info(
                    lowVoltageMessage.format(
                        GoPiGoRobot.LOW_VOLTAGE_THRESHOLD, voltage))
            return True
        return False

    def is_collision_imminent(self) -> None:
        '''Check if collison is imminent'''
        if not self.sensor_distance:
            raise IOError

        perimeter_in_cm = self.get_wheel_perimeter_in_cm()
        distance_in_mm = self.get_distance()
        speed_in_dps = self.get_speed(is_km_per_hr=False)
        speed_in_mm_per_sec = speed_in_dps * perimeter_in_cm * 10 / 360
        distance_to_go = speed_in_mm_per_sec * \
                RemoteRobot.DISTANCE_UPDATE_PERIOD * \
                RemoteRobot.COLLISION_AVOIDANCE_SAFETY_FACTOR
        nextDistance = round(distance_in_mm - distance_to_go)

        threshold = GoPiGoRobot.MIN_DISTANCE_IN_MM + \
                GoPiGoRobot.FRONTAL_SAFE_DISTANCE_IN_MM
        self.imminent_colllision = nextDistance < threshold
        if self.imminent_collision:
            message = 'Collision avoidance activated : threshold = {}, ' + \
                    'distance = {}, nextDistance = {}'
            logging.info(message.format(threshold, distance_in_mm, nextDistance))
        return self.imminent_collision

    def run(self, collision_avoidance=False, debug=False):
        try:
            self.initialize()
            self.manager_worker_threads(ThreadOperation.START)

            last_voltage_time = last_distance_time = time.time()
            while self.all_threads_alive():
                cur_time = time.time()

                time_voltage = cur_time - last_voltage_time
                if time_voltage > RemoteRobot.VOLTAGE_UPDATE_PERIOD:
                    # I/O read is slow. So we need to throttle the check frequencty
                    # and not to read the hardware too frequently
                    if self.is_low_voltage() and self.audio_player:
                        self.audio_player.speak('Low Voltage Warning')
                    last_voltage_time = cur_time

                time_distance = cur_time - last_distance_time
                if collision_avoidance and \
                        time_distance > RemoteRobot.DISTANCE_UPDATE_PERIOD:
                    # I/O read is slow. So we need to throttle the check frequencty
                    # and not to read the hardware too frequently
                    last_distance_time = cur_time
                    crashing = self.is_collision_imminent()
                    # We need to aquire the lock to gain the mutual exclusive access
                    # to the hardware.
                    self.acquire_lock()
                    if crashing and (self.current_motion == Direction.FORWARD):
                        self.gopigo.stop()
                        self.current_motion = Direction.STOP
                    self.release_lock()

                # The main thread do nothing.
                time.sleep(RemoteRobot.MAIN_LOOP_PERIOD)
        except SignalException as e:
            print(e)
            if debug:
                # Print traceback
                traceback.print_exc()
        finally:
            # Shutdown the threads gracefully
            self.manager_worker_threads(ThreadOperation.STOP)
            # Call thread join to clean up the thread resources
            self.manager_worker_threads(ThreadOperation.JOIN)
            # Stop the robot and reset all the servo motors
            self.reset()
            logging.info('Good Bye!')
