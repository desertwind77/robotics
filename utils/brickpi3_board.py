#!/usr/bin/env python3
'''A python class to manage a BrickPi board and its sensors and motors'''

from dataclasses import dataclass
import time
import sys

import brickpi3


SENSOR_PORTS = ['Port 1', 'Port 2', 'Port 3', 'Port 4']
MOTOR_PORTS = ['Port A', 'Port B', 'Port C', 'Port D']


@dataclass
class MotorStatus:
    '''A class to represent the status of a motor'''
    flags: int
    power: int
    encoder: int
    dps: int


@dataclass
class GyroStatus:
    '''A class to represent the status of a gyro sensor'''
    degree: int    # Absolute rotation
    rate: int      # Rotation rate


@dataclass
class RemoteStatus:
    '''A class to represent the status of a Infrared remote'''
    red_up: int
    red_down: int
    blue_up: int
    blue_down: int
    latch: int


class BrickPiEV3Base:
    '''The base class for everthing'''
    def get_port(self, port):
        '''Convert from the human-readable port to the enum value recognizable
        by BrickPi3'''
        if port == 'Port A':
            return brickpi3.BrickPi3.PORT_A
        if port == 'Port B':
            return brickpi3.BrickPi3.PORT_B
        if port == 'Port C':
            return brickpi3.BrickPi3.PORT_C
        if port == 'Port D':
            return brickpi3.BrickPi3.PORT_D
        if port == 'Port 1':
            return brickpi3.BrickPi3.PORT_1
        if port == 'Port 2':
            return brickpi3.BrickPi3.PORT_2
        if port == 'Port 3':
            return brickpi3.BrickPi3.PORT_3
        if port == 'Port 4':
            return brickpi3.BrickPi3.PORT_4
        assert False, f'Unknown port {port}'
        return None

    def get_sensor_type(self, sensor_type):
        '''Convert from the human-readable sensor type to the enum value
        recognizable by BrickPi3'''
        if sensor_type == 'Color':
            return brickpi3.BrickPi3.SENSOR_TYPE.EV3_COLOR_COLOR
        if sensor_type == 'Color Ambient':
            return brickpi3.BrickPi3.SENSOR_TYPE.EV3_COLOR_AMBIENT
        if sensor_type == 'Color Raw':
            return brickpi3.BrickPi3.SENSOR_TYPE.EV3_COLOR_COLOR_COMPONENTS
        if sensor_type == 'Color Reflected':
            return brickpi3.BrickPi3.SENSOR_TYPE.EV3_COLOR_REFLECTED
        if sensor_type == 'Gyro':
            return brickpi3.BrickPi3.SENSOR_TYPE.EV3_GYRO_ABS_DPS
        if sensor_type == 'Infrared':
            return brickpi3.BrickPi3.SENSOR_TYPE.EV3_INFRARED_PROXIMITY
        if sensor_type == 'Infrared Remote':
            return brickpi3.BrickPi3.SENSOR_TYPE.EV3_INFRARED_REMOTE
        if sensor_type == 'Touch':
            return brickpi3.BrickPi3.SENSOR_TYPE.TOUCH
        if sensor_type == 'Ultrasonic':
            return brickpi3.BrickPi3.SENSOR_TYPE.EV3_ULTRASONIC_CM
        assert False, f'Unknown sensor {sensor_type}'
        return None

    def get_status(self):
        '''Return the status of the device'''
        pass


class Motor(BrickPiEV3Base):
    '''The class to represent a motor'''

    def __init__(self, board, port):
        self.board = board
        self.port = self.get_port(port)

    def get_encoder(self):
        '''Get the encoder location'''
        try:
            result = self.board.get_motor_encoder(self.port)
        except IOError as error:
            print(error)
            return None
        return result

    def get_status(self):
        '''Get (flags, power, encoder, degress per sec)'''
        try:
            result = self.board.get_motor_status(self.port)
        except IOError as error:
            print(error)
            return None
        return MotorStatus(*result)

    def set_power(self, speed):
        '''Set the motor speed in percent between 0% and 100%'''
        speed = -100 if speed < -100 else speed
        speed = 100 if speed > 100 else speed
        try:
            self.board.set_motor_power(self.port, speed)
        except IOError as error:
            print(error)

    def set_limit(self, power=0, dps=0):
        '''Set the motor limit where 0 means no limit'''
        try:
            power = 0 if power < 0 else power
            power = 100 if power > 100 else power
            self.board.set_motor_limits(self.port, power, dps)
        except IOError as error:
            print(error)

    def set_degree_per_sec(self, degree_per_sec):
        '''Set the motor target speed in degree per sec'''
        try:
            self.board.set_motor_dps(self.port, degree_per_sec)
        except IOError as error:
            print(error)

    def offset_encoder(self, offset):
        '''Offset the encoder

        offset1 = self.board.get_motor_encoder(self.port)
        self.board.offset_motor_encoder(self.port, offset1)
        offset2 = self.board.get_motor_encoder(self.port)
        offset1 and offset2 are an interger, and 0, respectively
        '''
        try:
            self.board.offset_motor_encoder(self.port, offset)
        except IOError as error:
            print(error)

    def reset_encoder(self):
        '''Reset the encoder to 0'''
        self.offset_encoder(self.get_encoder())

    def set_position(self, target):
        '''Set the motor position. In other words, move the motor so tha its
        encoder matches the target'''
        try:
            self.board.set_motor_position(self.port, target)
        except IOError as error:
            print(error)

    def stop(self):
        '''Stop the motor'''
        self.set_power(brickpi3.BrickPi3.MOTOR_FLOAT)


class Sensor(BrickPiEV3Base):
    '''The class to represent a sensor'''

    def __init__(self, board, port, sensor_type):
        self.board = board
        self.port = self.get_port(port)
        self.sensor_type = self.get_sensor_type(sensor_type)
        self.board.set_sensor_type(self.port, self.sensor_type)

    def get_status(self):
        '''Return the sensor status

        Ultrasonic: return the distance in cm
        Infrared: return the infrared proximity. 100 is very close and anything
                  less than that means the object is further away
        Touoch: return 0 when the sensor is pressed; otherwise, return 1
        '''
        try:
            result = self.board.get_sensor(self.port)
        except brickpi3.SensorError:
            return None
        return result


class Gyro(Sensor):
    '''The class to represent the Gyro sensor'''

    def get_status(self):
        '''Return the gyro change degree and change rate'''
        result = super().get_status()
        return GyroStatus(*result) if result else None


class Color(Sensor):
    '''From my experiment, this sensor can't be used with Rubix because it
    cannot differentiate between red and orange'''

    def get_status(self):
        color = ["None", "Black", "Blue", "Green",
                  "Yellow", "Red", "White", "Brown"]
        result = super().get_status()
        return color[result] if result else None


class BrickPiEV3:
    '''The class to represent the BrickPi3 board'''

    def __init__(self, config):
        self.config = config

        try:
            self.board = brickpi3.BrickPi3()
        except brickpi3.FirmwareVersionError as error:
            print(error)
            sys.exit(1)
        self.devices = {}
        self.port_map = {}

        self.process_config()

    def process_config(self):
        motors = self.config['devices']['motors']
        for name, config in motors.items():
            self.add_motor(config['port'], name)

        sensors = self.config['devices']['sensors']
        for name, config in sensors.items():
            self.add_sensor(config['port'], name)

    def reset(self):
        '''Reset all the sensors and stop all the motors'''
        self.board.reset_all()

    def show_board_info(self):
        '''Show the hardware information'''
        try:
            print('Board:', self.board.get_board())
            print('Serial:', self.board.get_id())
            print('Manufacturer:', self.board.get_manufacturer())
            print('Hardware version:', self.board.get_version_hardware())
            print('Firmware version:', self.board.get_version_firmware())
            print('3.3V circuit voltage:', self.board.get_voltage_3v3())
            print('5V circuit voltage:', self.board.get_voltage_5v())
            print('9V circuit voltage:', self.board.get_voltage_9v())
            print('Battery voltage:', self.board.get_voltage_battery())
        except IOError as error:
            print(error)
            sys.exit(1)

    def show_device_info(self):
        '''Show all the sensors and motors' status'''
        for name, device in self.devices.items():
            print(f'{name}: {device.get_status()}')

    def set_led_brightness(self, level):
        '''Set the brightness level of the LED from 0 to 100'''
        level = 0 if level < 0 else level
        level = 100 if level > 100 else level
        try:
            self.board.set_led(level)
        except IOError as error:
            print(error)

    def add_sensor(self, port, sensor_type):
        '''Add a new sensor. We assume there will be one sensor for each sensor
        type. Once that assumption is no logner true, we need to revisit this
        code.'''
        assert port in SENSOR_PORTS, port
        if sensor_type in self.devices or \
                port in self.port_map:
            print(f'Either {sensor_type} or {port} are configured')
            return

        device = None
        if sensor_type in ['Color Ambient', 'Color Raw', 'Color Reflected',
                            'Infrared', 'Infrared Remote', 'Touch',
                            'Ultrasonic',]:
            device = Sensor(self.board, port, sensor_type)
        elif sensor_type == 'Gyro':
            device = Gyro(self.board, port, sensor_type)
        elif sensor_type == 'Color':
            device = Color(self.board, port, sensor_type)
        else:
            assert False, f'Unknown sensor type {sensor_type}'
        self.devices[sensor_type] = device
        self.port_map[port] = sensor_type

    def add_motor(self, port, name):
        '''Add a new motor'''
        if name in self.devices or \
                port in self.port_map:
            print(f'Either {name} or {port} are configured.')
            return
        self.devices[name] = Motor(self.board, port)
        self.port_map[port] = name

    def get_device(self, name):
        '''Get the device handler by name'''
        return self.devices.get(name)

    def get_sensor_status(self, sensor_type):
        '''Get the status of a sensor'''
        if sensor_type in self.devices:
            return self.devices[sensor_type].get_status()
        return None

    def get_gyro_status(self):
        '''Return the gyro sensor's status if connected'''
        return self.get_sensor_status('Gyro')

    def get_distance(self):
        '''Return the ultrasonic sensor's status in cm if connected'''
        return self.get_sensor_status('Ultrasonic')

    def get_touch_status(self):
        '''Return the touch sensor's status if connected'''
        return self.get_sensor_status('Touch')

    def get_color_status(self):
        '''Return the color sensor's status if connected'''
        sensor_type = None
        sensors = ['Color', 'Color Ambient', 'Color Raw', 'Color Reflected']
        for sensor_type in sensors:
            if sensor_type in self.devices:
                return self.get_sensor_status(sensor_type)
        return None

    def get_infrared_status(self):
        '''Return the Infrared sensor's status if connected'''
        return self.get_sensor_status('Infrared')

    def get_infrared_remote_status(self, channel):
        '''Return the Infrared remote sensor's status if connected'''
        key = self.get_sensor_status('Infrared Remote')
        if key:
            channel_key = key[channel - 1]
            return RemoteStatus(*channel_key)
        return None


def main() -> main:
    '''Demonstrate what BrickPi3 can do with EV3 motors and sensors'''
    robot = BrickPiEV3()

    motor_info = {
        'Motor B': 'Port B',
        'Motor C': 'Port C',
    }

    sensor_info = {
        # 'Gyro': 'Port 1',
        # 'Color': 'Port 2',
        # 'Color Ambient': 'Port 2',
        # 'Color Raw': 'Port 2',
        # 'Color Reflected': 'Port 2',
        # 'Infrared': 'Port 3',
        # 'Infrared Remote': 'Port 3',
        # 'Touch': 'Port 4',
        # 'Ultrasonic': 'Port 4',
    }

    try:
        # Add all the motors
        for motor, port in motor_info.items():
            robot.add_motor(port, motor)
        # Add all the sensors
        for sensor, port in sensor_info.items():
            robot.add_sensor(port, sensor)

        # Show the hardware information
        robot.show_board_info()

        # Dump the sensors and motors' status
        count = 0
        while count < 10:
            robot.show_device_info()
            time.sleep(1)
            count += 1

        # Given that the users rotate the motor B, the program will try to
        # match the motor C's encoder with that of the motor B.
        motor_b = robot.get_device('Motor B')
        motor_c = robot.get_device('Motor C')
        if motor_b and motor_c:
            motor_b.stop()
            motor_c.stop()

            motor_b.reset_encoder()
            motor_c.reset_encoder()

            motor_c.set_limit(power=50, dps=200)
            while True:
                target = motor_b.get_encoder()
                if target is not None:
                    motor_c.set_position(target)
                    pos = motor_c.get_encoder()
                    print(f'Left motor: {target}, Right motor: {pos}')
    except KeyboardInterrupt:
        # Gracefully terminate the program when the users presss Ctrl-C
        pass
    finally:
        # Unconfigure the sensors, disable the motors and relinquish
        # the control of the LED to the firmware
        robot.reset()


if __name__ == '__main__':
    main()
