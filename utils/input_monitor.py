#!/usr/bin/env python3
''''''
from selectors import DefaultSelector, EVENT_READ
import evdev
import logging
import select
import socket
import time

from utils import ThreadBase


class InputMonitor(ThreadBase):
    '''
    Processing input from USB keyboard, gamepad, and network
    '''
    # The server IP address
    LOCAL_IP = '127.0.0.1'
    # The server port
    LOCAL_PORT = 8001
    # The maximum time to process the input from an USB device.
    # Yield the control back after we stay in the loop to process
    # the input to prevent starvation on other input devices.
    INPUT_PROCESSING_PERIOD = 0.08

    def __init__(self, robot, audioplayer) -> None:
        '''Constructor

        Args:
           robot: the robot hardware board

           audioplayer: the AudioPlayer object to play audio over bluetooth
        '''
        super(InputMonitor, self).__init__()
        self.name = 'InputMonitor'
        # The robot hardware board
        self.robot = robot
        # The audio player object
        self.audio_player = audioplayer
        # Unix file selector for input multiplexing
        self.selector = DefaultSelector()
        # Known devices connected to the system
        self._knownDevice = set()
        # The network socket for receiving the control over the network
        self.socket = None

        # Scan and register the USB input devices connected to the system
        self.scan_input_devices()
        # Set up the UDP server to receive the control from the network
        self.setup_udp_server()

    def scan_input_devices(self) -> None:
        '''Scan and register the USB input devices connected to the system'''
        for path in evdev.list_devices():
            device = evdev.InputDevice(path)

            if path in self._knownDevice:
                continue
            if not self.is_joypad(device) and not self.is_keyboard(device):
                continue

            logging.info("{} registering {} {} {}".format(self.name,
                         device.path, device.name, device.phys))

            self.selector.register(device, EVENT_READ)
            self._knownDevice.add(path)

    def setup_udp_server(self) -> None:
        '''Set up the UDP server to receive the control from the network'''
        self.socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.socket.bind((InputMonitor.LOCAL_IP, InputMonitor.LOCAL_PORT))
        # Make the socket non-blocking
        self.socket.setblocking(0)
        self.selector.register(self.socket, EVENT_READ)

    def is_joypad(self, device) -> bool:
        '''Check if the device is a USB joypad. This function works only with
        the Logitech USB gamepad that we are using. If a new joypad is used,
        the implementation may need to be updated.
        '''
        return ('Gamepad' in device.name) or ('RumblePad' in device.name)

    def is_keyboard(self, device) -> bool:
        '''Check if the device is a USB keyboard. This function works only with
        the keyboard that we are using. If a new joypad is used, the
        implementation may need to be updated.
        '''
        return ('HID' in device.name) and \
               ('System Control' not in device.name) and \
               ('Mouse' not in device.name) and \
               ('Consumer Control' not in device.name)

    def process_gamepad_input(self, device) -> None:
        '''Handle the input from the USB keyboard

        Args:
           device: the input device
        '''
        startTime = time.time()
        for event in device.read():
            # We don't want to stuck in the tight loop for too long
            curTime = time.time()
            if curTime - startTime > self.INPUT_PROCESSING_PERIOD:
                return

            if event.type == 0 and event.code == 0 and event.value == 0:
                continue

            if event.type == evdev.ecodes.EV_ABS:       # Analog gamepad
                absevent = evdev.categorize(event)
                absevent_code = \
                        evdev.ecodes.bytype[absevent.event.type][absevent.event.code]
                absevent_value = absevent.event.value

                logging.debug("{} gamepad {}.{} {} {}".format(self.name,
                    event.sec, event.usec, absevent_code, absevent_value))

                if absevent_code in ['ABS_HAT0X', 'ABS_HAT0Y']:
                    # The left dpad which controls the robot movement
                    if absevent_code == 'ABS_HAT0X':
                        if absevent.event.value > 0:         # dpad right
                            self.robot.right()
                        elif absevent.event.value < 0:       # dpad left
                            self.robot.left()
                        else:                                # dpad release
                            self.robot.stop()
                    elif absevent_code == 'ABS_HAT0Y':
                        if absevent.event.value > 0:        # dpad down
                            self.robot.backward()
                        elif absevent.event.value < 0:      # dpad up
                            self.robot.forward()
                        else:                               # dpad release
                            self.robot.stop()
                elif absevent_code in ['ABS_RX', 'ABS_RY']:
                    # The right joystick which control the servo motors
                    # that controls the camera.

                    # ABS_RX and ABS_RY control pan and tilt, respectively.
                    is_panning = absevent_code == 'ABS_RX'
                    default_position = \
                            self.robot.get_default_servo_pos(is_pan=is_panning)
                    # Convert the analog input value from our joypad to the
                    # degress that the hardware expect
                    value = int(abs(absevent_value) * 90 / 32767)
                    diff = -value if absevent_value < 0 else value
                    pos = default_position + diff

                    if is_panning:
                        self.robot.pan(pos)
                    else:
                        self.robot.tilt(pos)
                elif absevent_code in ['ABS_X', 'ABS_Y']:
                    # The left joystick. Unused for now.
                    pass
                elif absevent_code in ['ABS_Z', 'ABS_RZ']:
                    # The analog left and right triggers. Unused for now.
                    pass
            elif event.type == evdev.ecodes.EV_KEY:
                # Handle buttons on the gamepad
                key = evdev.ecodes.keys[event.code]

                logging.debug("{} gamepad {}.{} {}".format(self.name,
                    event.sec, event.usec, key))

                if event.value == 1:
                    if isinstance(key, str) and key in ['BTN_SELECT', 'BTN_START']:
                        # Control the pan servo mother
                        diff = -10 if key == 'BTN_SELECT' else 10
                        pos = self.robot.get_servo_pos(is_pan=True) + diff
                        self.robot.pan(pos)
                    elif isinstance(key, str) and key in ['BTN_TL', 'BTN_TR']:
                        # Control the tilt servo mother
                        diff = -10 if key == 'BTN_TR' else 10
                        pos = self.robot.get_servo_pos(is_pan=False) + diff
                        self.robot.tilt(pos)
                    elif isinstance(key, str) and key in ['BTN_THUMBL', 'BTN_THUMBR']:
                        # Control the robot acceleration
                        if key == 'BTN_THUMBL':
                            self.robot.decelerate()
                        else:
                            self.robot.accelerate()
                    elif isinstance(key, list) and any(['BTN_A' in key, 'BTN_B' in key,
                                                        'BTN_X' in key, 'BTN_Y' in key]):
                        # Control the audio plyaer
                        if not self.audio_player:
                            continue
                        if 'BTN_A' in key:
                            self.audio_player.play_library(0)
                        elif 'BTN_B' in key:
                            self.audio_player.play_library(1)
                        elif 'BTN_X' in key:
                            self.audio_player.play_library(2)
                        elif 'BTN_Y' in key:
                            self.audio_player.stop()

    def process_keyboard_input(self, device) -> None:
        '''Handle the input from the USB keyboard

        Args:
           device: the input device
        '''
        startTime = time.time()
        for event in device.read():
            curTime = time.time()
            if curTime - startTime > self.INPUT_PROCESSING_PERIOD:
                # If we stuck in the tight loop processing only one type of
                # input for too long, we may cause starvation on the other
                # input devices. So we will yield the control back after
                # staying in this loop for a certain period of time.
                return

            if event.type == 0 and event.code == 0 and event.value == 0:
                # Ignore bogus input
                continue

            key = evdev.ecodes.KEY[event.code]
            if event.value == 0:
                # The button was released.
                self.robot.stop()
            else:
                # A button was pressed.
                if key in ['KEY_UP', 'KEY_DOWN', 'KEY_LEFT', 'KEY_RIGHT']:
                    logging.debug("{} keyboard {}.{} {}".format(self.name,
                        event.sec, event.usec, key))

                if key == 'KEY_UP':
                    self.robot.forward()
                elif key == 'KEY_DOWN':
                    self.robot.backward()
                elif key == 'KEY_LEFT':
                    self.robot.left()
                elif key == 'KEY_RIGHT':
                    self.robot.right()

    def process_network_input(self, device) -> None:
        '''Handle the input from the web interface over network

        Args:
            device: the network socket to read from
        '''
        (message, address) = device.recvfrom(1024)
        message = bytes.decode(message)
        message = message.rstrip()

        logging.debug("{} address: {} message: {}".format(self.name,
            address, message))

        if message == 'forward':
            self.robot.forward()
        elif message == 'backward':
            self.robot.backward()
        elif message == 'left':
            self.robot.left()
        elif message == 'right':
            self.robot.right()
        elif message == 'stop':
            self.robot.stop()
        elif message in ['pan_left', 'pan_right']:
            diff = 10 if message == 'pan_left' else -10
            pos = self.robot.getPanPos() + diff
            self.robot.pan(pos)
        elif message in ['tilt_down', 'tilt_up']:
            diff = 10 if message == 'tilt_down' else -10
            pos = self.robot.getTiltPos() + diff
            self.robot.tilt(pos)
        elif 'speed=' in message:
            speed = message[len('speed='):]
            speed = int(speed)
            self.robot.setSpeed(speed)

    def run(self) -> None:
        '''The main loop of the thread'''
        while not self.isExiting.is_set():
            try:
                # Select the file description for input every 1 sec.
                #
                # If we are interested in only one device, we can use
                # for event in device.read_loop(). But blocking call won't
                # play well with thread's gracefull exit.
                for key, mask in self.selector.select(timeout=1):
                    device = key.fileobj
                    if isinstance(device, socket.socket):
                        # Handle network input
                        self.process_network_input(device)
                    elif self.is_joypad(device):
                        # Handle a USB joypad input
                        self.process_gamepad_input(device)
                    elif self.is_keyboard(device):
                        # Handle a USB keyboard input
                        self.process_keyboard_input(device)
            except Exception as e:
                logging.error(e)
                return

    def help(self) -> None:
        '''Print the help message'''
        msg = '''
Keyboard:
   Up                      Move forward
   Down                    Move backward
   Left                    Turn left
   Rigth                   Turn right

Gamepad
   DPad Up                 Move forward
   DPad Down               Move backward
   DPad Left               Turn left
   DPad Rigth              Turn right
   Left Stick Up           Unused
   Left Stick Down         Unused
   Left Stick Left         Unused
   Left Stick Right        Unused
   Right Stick Up          Tilt up
   Right Stick Down        Tilt down
   Right Stick Left        Pan left
   Right Stick Right       Pan right
   Select                  Pan left for 10 degree
   Start                   Pan rigth for 10 degree
   Left Trigger            Unused (analog)
   Left Shoulder           Tilt down for 10 degrees
   Rigth Trigger           Unused (analog)
   Right Shoulder          Tilt up for 10 degrees
   A                       Stop the current song and play the song #1
   B                       Stop the current song and play the song #2
   X                       Stop the current song and play the song #3
   Y                       Stop playing the song
'''
        print(msg)
