#!/usr/bin/env python3
from collections import namedtuple
from curtsies import Input
import logging
import sys
import threading
import time
import traceback

from utils import install_signal_handler, SignalException, ThreadBase
from utils.gopigo3_board import Direction, GoPiGoRobot


Key = namedtuple('key', 'key func descr')


class PIDRobot(GoPiGoRobot):
    '''
    A line-following robot using a PID controller

    Note that for this robot, _currentMotion is either FORWARD or STOP
    '''

    INIT_SPEED = 200

    def __init__(self) -> None:
        super(PIDRobot, self).__init__()
        self.pid_control_lock = threading.Lock()
        # 0.5  : the line center is in the middle of the line
        # <0.5 : the line center is on the left side of the line
        # >0.5 : the line center is on the right side of the line
        self.target_position = 0.5
        self.kp = 1000.0
        self.ki = 0.0
        self.kd = 0.0
        self.integral_area = 0.0
        self.previous_error = 0.0
        self.pid_update_freq = 100
        self.left_motor_speed = 0
        self.right_motor_speed = 0

    def lock_pid(origFunc):
        '''A decorator to acquire and release the pid_control_lock'''
        def wrapper(self, *args, **kwargs):
            result = None
            self.pid_control_lock.acquire()
            result = origFunc(self, *args, **kwargs)
            self.pid_control_lock.release()
            return result
        return wrapper

    @lock_pid
    def set_kp(self,value: int) -> None:
        '''Set the Kp value'''
        self.kp = value

    @lock_pid
    def get_kp(self) -> int:
        '''Get the Kp value'''
        return self.kp

    @lock_pid
    def set_ki(self, value: float) -> None:
        '''Set the Ki value'''
        self.ki = value

    @lock_pid
    def get_ki(self) -> float:
        '''Get the Ki value'''
        return self.ki

    @lock_pid
    def set_kd(self, value) -> None:
        '''Set the Kd value'''
        self.kd = value

    @lock_pid
    def get_kd(self) -> float:
        '''Get the Kd value'''
        return self.kd

    @lock_pid
    def set_error_area(self, value: float) -> None:
        '''Set the error area'''
        self.integral_area = value

    @lock_pid
    def get_error_area(self) -> float:
        '''Get the error area'''
        return self.integral_area

    @lock_pid
    def set_update_freq(self, value) -> None:
        '''Set the update freq'''
        self.pid_update_freq = value

    @lock_pid
    def get_update_freq(self) -> float:
        '''Get the update freq'''
        return self.pid_update_freq

    @lock_pid
    def get_left_motor_speed(self):
        '''Get the left motor speed'''
        return self.left_motor_speed

    @lock_pid
    def get_right_motor_speed(self):
        '''Get the right motor speed'''
        return self.right_motor_speed

    @lock_pid
    def do_line_following(self) -> None:
        '''Walk the line and adjust the left and right motor speeds based on
        how far the robot is from the middle of the line'''
        begin_time = time.time()
        update_period = 1.0 / self.pid_update_freq

        current_position, _ = self.get_line_position(weightedAvg=True)

        # Calculate the correction we have to make using PID
        current_error = current_position - self.target_position
        if self.ki < 0.0001 and self.ki > -0.0001:
           self.integral_area = 0.0
        else:
           self.integral_area += current_error
        correction = (self.kp * current_error) + \
                     (self.ki * self.integral_area) + \
                     (self.kd * (current_error - self.previous_error))
        self.previous_error = current_error

        self.left_motor_speed = max(1, int(self.current_speed + correction))
        self.right_motor_speed = max(1, int(self.current_speed - correction))

        if self.current_motion != Direction.STOP:
           debugMsg = 'Pos: {} Error: {} Correction: {} Left: {} Right: {}'
           debugMsg = debugMsg.format(current_position, current_error, correction,
                                       self.left_motor_speed, self.right_motor_speed)
           if current_position != self.target_position:
              logging.debug(debugMsg)

           self.set_motor_speed(self.left_motor_speed, is_left=True)
           self.set_motor_speed(self.right_motor_speed, is_left=False)

        endTime = time.time()
        elapsedTime = endTime - begin_time
        if elapsedTime < update_period:
           time.sleep(update_period - elapsedTime)


class RobotControlThread(ThreadBase):
    '''The threa that moves the robot along the line while the main thread is
    responsible for receiving the user input and adjust the PID parameter in
    real time.'''

    def __init__(self, robot) -> None:
        '''Constructor'''
        super(RobotControlThread, self).__init__()
        self.name = 'RobotControlThread'
        self.robot = robot
        self.thread_exception = None

    def run(self) -> None:
        '''The main loop of the thread'''
        try:
            while not self.is_exiting.is_set():
                self.robot.do_line_following()
        except:
            self.thread_exception = sys.exc_info()

    def join(self) -> None:
        '''Overrid the join method to print the exception that terminated the thread'''
        threading.Thread.join(self)
        if self.thread_exception:
            msg = "Thread {} threw an exception: {}".format(
                    self.name, self.thread_exception[1])
            new_exception = Exception(msg)
            raise new_exceptoin.with_traceback(self.thread_exception[2])


class Robot(object):
    '''The complete robot class that have both the base board with PID line
    following and the RobotControlThread'''

    INIT_SPEED = 200
    STEP_SPEED = 10
    STEP_FREQ = 1
    STEP_KP = 5
    STEP_KI = 0.001
    STEP_KD = 100.0

    def __init__(self) -> None:
        '''Constructure'''
        self.key_binding = [
            Key('b', 'calibrate_black', 'Set the black point for line sensor'),
            Key('w', 'calibrate_white', 'Set the white point for line sonsor'),
            Key('u', 'inc_kp_gain', 'Increase the Kp gain'),
            Key('j', 'dec_kp_gain', 'Decrease the Kp gain'),
            Key('i', 'inc_ki_gain', 'Increase the Ki gain'),
            Key('k', 'dec_ki_gain', 'Decrease the Ki gain'),
            Key('o', 'inc_kd_gain', 'Increase the Kd gain'),
            Key('l', 'dec_kd_gain', 'Decrease the Kd gain'),
            Key('r', 'reset_error_area', 'Reset the error area (integral) to 0.0'),
            Key('1', 'inc_pid_sampling_freq', 'Increase the PID update frequency'),
            Key('2', 'dec_pid_sampling_freq', 'Decrease the PID update frequency'),
            Key('3', 'inc_speed', 'Increase the robot speed'),
            Key('4', 'dec_speed', 'Decrease the robot speed'),
            Key('g', 'start', 'Start the robot'),
            Key('s', 'stop', 'Stop the robot'),
            Key('q', 'exit', 'Exit the program'),
        ]
        self.dispatch_table = {}
        self.input_rate = 20
        self.running = False
        self.robot = None
        self.robot_control_thread = None

    def initialize(self) -> None:
        '''Initialize the dispatch table to handle the user input, create
        robot instance and the robot control thread.'''
        # Set up the dispatch table from the key binding
        for k in self.key_binding:
            self.dispatch_table[ k.key ] = getattr(self, k.func)

        # Print the key binding
        for k in self.key_binding:
            print('{}\t{}'.format(k.key, k.descr))
        print()

        self.robot = PIDRobot()
        self.robot.set_speed(Robot.INIT_SPEED)
        self.robot_control_thread = RobotControlThread(self.robot)
        self.robot_control_thread.start()
        # Enble the main input loop
        self.running = True

    def cleanup(self) -> None:
        '''Stop the robot and shutdown the robot control thread'''
        self.robot.stop()
        if self.robot_control_thread:
            self.robot_control_thread.shutdown()
            self.robot_control_thread.join()

    def dispatch(self, key: str) -> None:
        '''Handle the user input from keyboard'''
        if not key:
            return

        if (func := self.dispatch_table.get(key, None)):
            func()
            self.print_current_state()

    def print_current_state(self) -> None:
        '''Print the current Robot state especially the PID value'''
        kp = self.robot.get_kp()
        ki = self.robot.get_ki()
        kd = self.robot.get_kd()
        cur_speed = self.robot.get_speed(is_km_per_hr=False)
        left_speed = self.robot.get_left_motor_speed()
        right_speed = self.robot.get_right_motor_speed()
        error_area = self.robot.get_error_area()
        freq = self.robot.get_update_freq()

        msg = 'Kp={:3f} Ki={:3f} Kd={:3f} ' + \
              'Speed={:3d} Left={:3d} Right={:3d} ' + \
              'ErrorArea={:3f} UpdateFreq={:3d}'
        msg = msg.format(kp, ki, kd, cur_speed, left_speed,
                         right_speed, error_area, freq)
        logging.debug(msg)

    def calibrate_black(self) -> None:
        '''Calibrate the line following sensor to black'''
        self.robot.calibrate('black')

    def calibrate_white(self) -> None:
        '''Calibrate the line following sensor to white'''
        self.robot.calibrate('white')

    def inc_kp_gain(self) -> None:
        '''Increase Kp gain'''
        cur_kp = self.robot.get_kp()
        self.robot.set_kp(cur_kp + Robot.STEP_KP)

    def dec_kp_gain(self) -> None:
        '''Decrease Kp gain'''
        cur_kp = self.robot.get_kp()
        self.robot.set_kp(cur_kp - Robot.STEP_KP)

    def inc_ki_gain(self) -> None:
        '''Increase Ki gain'''
        cur_ki = self.robot.get_ki()
        self.robot.set_ki(cur_ki + Robot.STEP_KI)

    def dec_ki_gain(self) -> None:
        '''Decrease Ki gain'''
        cur_ki = self.robot.get_ki()
        self.robot.set_ki(cur_ki - Robot.STEP_KI)

    def inc_kd_gain(self) -> None:
        '''Increase Kd gain'''
        cur_kd = self.robot.get_kd()
        self.robot.set_kd(cur_kd + Robot.STEP_KD)

    def dec_kd_gain(self) -> None:
        '''Decrease Kd gain'''
        cur_kd = self.robot.get_kd()
        self.robot.set_kd(cur_kd - Robot.STEP_KD)

    def reset_error_area(self) -> None:
        '''Reset the error area'''
        self.robot.set_error_area(0.0)

    def inc_pid_sampling_freq(self) -> None:
        '''Increase the sampling frequency'''
        curFreq = self.robot.get_update_freq()
        self.robot.set_update_freq(curFreq + Robot.STEP_FREQ)

    def dec_pid_sampling_freq(self) -> None:
        '''Decrease the sampling frequency'''
        curFreq = self.robot.get_update_freq()
        self.robot.set_update_freq(curFreq - Robot.STEP_FREQ)

    def inc_speed(self) -> None:
        '''Decrease speed'''
        cur_speed = self.robot.get_speed(is_km_per_hr=False)
        self.robot.set_speed(cur_speed + Robot.STEP_SPEED)

    def dec_speed(self) -> None:
        '''Decrease speed'''
        cur_speed = self.robot.get_speed(is_km_per_hr=False)
        self.robot.set_speed(cur_speed - Robot.STEP_SPEED)

    def start(self) -> None:
        '''Start following the line'''
        self.robot.forward()

    def stop(self) -> None:
        '''Stop the robot'''
        self.robot.stop()

    def exit(self) -> None:
        '''Exit the program'''
        self.running = False

    def run(self, debug: bool = False) -> None:
        try:
            self.initialize()
            self.print_current_state()

            inputPeriod = 1.0 / self.input_rate
            with Input(keynames="curtsies", sigint_event = True) as keyInput:
                while self.running:
                    # This is similar to the select system call which waits for the keyboard
                    # input for inputPeriod. None is return in case of no input.
                    key = keyInput.send(inputPeriod)
                    self.dispatch(key)

                    # Check if the robot_control_thread thread caught any exception
                    # Note that self.robot_control_thread should not be None. But I check
                    # for None here because sometimes I set it to None for debugging
                    if self.robot_control_thread and not self.robot_control_thread.isAlive():
                        self.robot_control_thread.join()
                        del self.robot_control_thread
                        self.robot_control_thread = None
        except Exception as e:
            print(e)
            if debug:
                # Print traceback
                traceback.print_exc()
        finally:
            self.cleanup()
