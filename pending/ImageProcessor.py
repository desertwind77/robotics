#!/usr/bin/env python3
from collections import namedtuple
import cv2
import io
import os
import numpy
import picamera
import time

from GoPiGo import GoPiGoRobot
from RobotUtil import installSignalHandler

Face = namedtuple( 'Face', 'x y w h' )

class ImageProcessor( object ):
   FACE_CASECADE = 'cascades/haarcascade_frontalface_default.xml'

   def getCascade( self ):
      curDir = os.getcwd()
      fullPath = os.path.join( curDir, self.FACE_CASECADE )
      return fullPath

   def captureFrame( self, count ):
      stream = io.BytesIO()
      with picamera.PiCamera() as camera:
         # Resolution      FPS
         # 320 x 240       60
         # 640 x 480       46
         # 1280 x 960      27
         # 1440 x 1080     26
         camera.resolution = ( 320, 240 )
         camera.capture( stream, format='jpeg' )

      # Convert the picture into a numpy array
      buff = numpy.frombuffer( stream.getvalue(), dtype=numpy.uint8 )
      # Create an OpenCV image for further processing
      image = cv2.imdecode( buff, 1 )

      faceCascade = cv2.CascadeClassifier( self.getCascade() )
      # Convert the image to grayscale cause we don't use color
      gray = cv2.cvtColor( image, cv2.COLOR_BGR2GRAY )
      # Detect faces in the image
      faces = faceCascade.detectMultiScale( gray, 1.1, 5 )

      # We only care about the biggest face
      maxArea  = 0
      maxFace = None
      for ( x, y, w, h ) in faces:
         curArea = w * h
         if curArea > maxArea:
            maxArea = curArea
            maxFace = Face( x, y, w, h )

      if maxFace:
         filename = 'result' + str( count ) + '.jpg'
         #cv2.rectangle( image, ( x, y ), ( x + w, y + h ), ( 255, 255, 0 ), 2 )
         #cv2.imwrite( filename, image )

      return maxFace

def doMove( robot, lastFace, curFace ):
   lastArea = curArea = 0
   if lastFace:
      lastArea = lastFace.w * lastFace.h
   if curFace:
      curArea = curFace.w * curFace.h

   if lastArea and curArea:
      distance = robot.getDistance()
      if lastArea > curArea:
         print( 'x', end='', flush=True )
         if distance > 30:
            robot.forward()
            time.sleep( 1 )
            robot.stop()
      # elif lastArea < curArea:
      #    print( 'y', end='', flush=True )
      #    if distance < 3000:
      #       robot.backward()
      #       time.sleep( 1 )
      #       robot.stop()
   elif lastArea and not curArea:
      curMotion = robot.getCurrentMotion()
      if curMotion  == 'Backward':
         print( '.', end='', flush=True )
         robot.forward()
         time.sleep( 1 )
         robot.stop()
      elif curMotion == 'Forward':
         print( '_', end='', flush=True )
         robot.backward()
         time.sleep( 1 )
         robot.stop()
      else:
         print( ' ', end='', flush=True )
   elif not lastArea and curArea:
      pass

def loop( imageProcessor, robot ):
   print( '----- START -----' )
   startTime = curTime = time.time()
   count = 0
   throttle = 0
   lastFace = None
   while True:
      curTime = time.time()
      if curTime - startTime > 1 * 60:
         break

      curFace = imageProcessor.captureFrame( count )
      doMove( robot, lastFace, curFace )
      lastFace = curFace
      count += 1

def main():
   installSignalHandler()
   imageProcessor = ImageProcessor()
   robot = GoPiGoRobot()

   try:
      loop( imageProcessor, robot )
   except Exception as e:
      print( e )
   finally:
      print()
      robot.stop()

main()
