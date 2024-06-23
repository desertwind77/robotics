#!/usr/bin/env python3
'''
A ball tracking robot

TODO
- Pan back and forth when the object is near
- Livestream
'''

import logging
import math
import os
import time
import traceback

from imutils.video import VideoStream
import cv2
import imutils

from utils import install_signal_handler, setup_logging
from utils.gopigo3_board import GoPiGoRobot

ball_data = {
    "Blue": {
        "lower": (89, 100, 100),
        "upper": (109, 255, 255)
    },
    "Green": {
        "lower": (61, 100, 100),
        "upper": (81, 255, 255)
    }
}


class BallTrackingRobot(GoPiGoRobot):
    MIN_BALL_RADIUS = 10
    MAX_BALL_RADIUS = 60
    CENTER_FRAME_COLOR = (0, 0, 255)
    CENTER_FRAME_WIDTH = 100
    CENTER_FRAME_HEIGHT = 100
    FRAME_WIDTH = 500
    FRAME_HEIGHT = 360
    PAN_ADJUST = 5
    TILT_ADJUST = 3
    Kp = 10

    def __init__(self, color: str) -> None:
        '''Constructor'''
        assert color in ball_data

        super(BallTrackingRobot, self).__init__()
        self.color = color

        # Determine the lower and upper bounds of the color of choice
        self.lower_bound = ball_data[color]['lower']
        self.upper_bound = ball_data[color]['upper']

        step_x = (BallTrackingRobot.FRAME_WIDTH - BallTrackingRobot.CENTER_FRAME_WIDTH) / 2
        self.min_x = step_x
        self.max_x = BallTrackingRobot.FRAME_WIDTH - step_x

        step_y = (BallTrackingRobot.FRAME_HEIGHT - BallTrackingRobot.CENTER_FRAME_HEIGHT) / 2
        self.min_y = step_y
        self.max_y = BallTrackingRobot.FRAME_HEIGHT - step_y

        # Start the camera
        self.camera = VideoStream(0).start()


    def get_video_frame(self):
        '''Capture a frame from the camera and return the frame in BGR and HSV'''
        frame_in_bgr = self.camera.read()
        if frame_in_bgr is None:
            raise Exception('Unable to read from the camera')
        frame_in_bgr = imutils.resize(frame_in_bgr, width=BallTrackingRobot.FRAME_WIDTH)
        #frame_in_bgr = imutils.rotate(frame_in_bgr, angle=180)
        frame_in_hsv = cv2.cvtColor(frame_in_bgr, cv2.COLOR_BGR2HSV)
        return (frame_in_bgr, frame_in_hsv)

    def get_mask(self, frame_in_hsv):
        '''Based on the lower and upper bounds, convert the frame_in_hsv into a mask'''
        # Construct a mask for the object color, then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(frame_in_hsv, self.lower_bound, self.upper_bound)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        return mask

    def get_contour(self, mask):
        '''Retrieve the largest contour in the frame'''
        contour = None
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
        #contours = contours[0] if imutils.is_cv2() else contours[1]
        contours = contours[0]
        if len(contours) > 0:
            contour = max(contours, key=cv2.contourArea)
        return contour

    def move(self, x, y, radius):
        '''
        Move the camera and the robot based on the position of the ball
        in the frame.
        '''
        # Handling pan. The x range is between 1 and 500
        panAdjust = 0
        if x < self.min_x:
            panAdjust = BallTrackingRobot.PAN_ADJUST
        elif x > self.max_y:
            panAdjust = -1 * BallTrackingRobot.PAN_ADJUST

        # Handling tilt. The y range is between 1 and 360
        tiltAdjust = 0
        if y < self.min_y:
            tiltAdjust = -1 * BallTrackingRobot.TILT_ADJUST
        elif y > self.max_y:
            tiltAdjust = BallTrackingRobot.TILT_ADJUST

        self.pan(self.get_servo_pos(is_pan=True) + panAdjust)
        self.tilt(self.get_servo_pos(is_pan=False) + tiltAdjust)

        left_motor_speed = 0
        right_motor_speed = 0
        if radius > BallTrackingRobot.MIN_BALL_RADIUS  and radius < BallTrackingRobot.MAX_BALL_RADIUS:
            panAngle = self.get_servo_pos(is_pan=True) - BallTrackingRobot.DEFAULT_PAN_POS
            correction = self.Kp * panAngle
            left_motor_speed = max(1, int(self.current_speed - correction))
            right_motor_speed = max(1, int(self.current_speed + correction))
        self.set_motor_speed(left_motor_speed, is_left=True)
        self.set_motor_speed(right_motor_speed, is_left=False)

        msg = f'x = {x}, y = {y}, raius = {radius}, left_motor_speed = {left_motor_speed}, ' + \
              f'right_motor_speed = {right_motor_speed}'
        logging.info(msg)

    def stop(self):
        self.set_motor_speed(0, is_left=True)
        self.set_motor_speed(0, is_left=False)

    def run(self, display_on_screen: bool = False, debug: bool = False):
        # Install the signal handler so that we can terminate the program
        # gracefully with Ctrl-C
        install_signal_handler()
        # Setup logging
        setup_logging()
        logging.info('Started')

        try:
            while True:
                (frame_in_bgr, frame_in_hsv) = self.get_video_frame()
                mask = self.get_mask(frame_in_hsv)
                contour = self.get_contour(mask)

                if contour is not None:
                    # Find the largest contour in the mask, then use it
                    # to compute the mininum enclosing circle and centroid
                    ((x, y), radius) = cv2.minEnclosingCircle(contour)
                    M = cv2.moments(contour)
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                    if radius > BallTrackingRobot.MIN_BALL_RADIUS:
                        x = int(x)
                        y = int(y)
                        radius = int(radius)
                        cv2.circle(frame_in_bgr, (x, y), radius, BallTrackingRobot.CENTER_FRAME_COLOR, 2)
                        cv2.circle(frame_in_bgr, center, 5, BallTrackingRobot.CENTER_FRAME_COLOR, -1)
                        self.move(x, y, radius)

                    try:
                        if display_on_screen:
                            cv2.imshow("Frame", frame_in_bgr)
                    except cv2.error:
                        display_on_screen = False
                else:
                    self.stop()
        except Exception as e:
            print(e)
            if debug:
                # Print traceback
                traceback.print_exc()
        finally:
            cv2.destroyAllWindows()
            # Stop the camera
            self.camera.stop()
            # Stop the robot
            self.reset()
