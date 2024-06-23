#!/usr/bin/env python3
# This is a simple script to demonstrate GoPiGo robot using
# the GoPiGo3 SDK.

import time

from easygopigo3 import EasyGoPiGo3
from di_sensors.easy_line_follower import EasyLineFollower

def main():
   # Create an GoPiGo3 obejct
   gopigo = EasyGoPiGo3( use_mutex=True )
   # Create an object for the servo motor that controls the tilt movement
   # This motor is connected to the port called SERV01.
   tilt = gopigo.init_servo( 'SERVO1')
   # Create an object for the servo motor that controls the pan movement
   # This motor is connected to the port called SERV02.
   pan = gopigo.init_servo( 'SERVO2')

   # Move the robot
   gopigo.forward()
   time.sleep( 1 )
   gopigo.left()
   time.sleep( 1 )
   gopigo.right()
   time.sleep( 1 )
   gopigo.backward()
   time.sleep( 1 )
   gopigo.stop()

   # Pan the servor motor which holds the Pi camera 40 and
   # 125 degrees, respectively.
   pan.rotate_servo( 40 )
   time.sleep( 1 )
   pan.rotate_servo( 125 )
   time.sleep( 1 )

   # Tilt the servor motor which holds the Pi camera 45 and
   # 120 degrees, respectively.
   tilt.rotate_servo( 45 )
   time.sleep( 1 )
   tilt.rotate_servo( 120 )
   time.sleep( 1 )

if __name__ == '__main__':
   main()
