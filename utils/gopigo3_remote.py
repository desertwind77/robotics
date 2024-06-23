#!/usr/bin/python3
# A class for collion avoidance robot

from enum import Enum
import logging
import time
import traceback

from utils import install_signal_handler, SignalException
from utils.audio_player import AudioPlayer
from utils.input_monitor import InputMonitor
from utils.gopigo3_board import GoPiGoRobot


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

