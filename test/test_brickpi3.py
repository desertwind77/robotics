#!/usr/bin/env python3

import time
from bp01 import BrickPiEV3

motor_info = {
    'Motor B' : 'Port B',
    'Motor C' : 'Port C',
}

try:
    robot = BrickPiEV3()
    for motor, port in motor_info.items():
        robot.add_motor( port, motor )

    motor_b = robot.get_device( 'Motor B' )
    motor_c = robot.get_device( 'Motor C' )

    power = 100
    motor_b.set_power( power )
    motor_c.set_power( power )
    while True:
        time.sleep( 1 )
except KeyboardInterrupt:
    pass
finally:
    robot.reset()
