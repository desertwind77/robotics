#!/usr/bin/env python3
'''
Web Service for the GoPiGo robot. It has two functionalities.
1) Deliver a live video stream
2) GUI for remote robot control

Source code from the official PiCamera package
http://picamera.readthedocs.io/en/latest/recipes2.html#web-streaming
'''

from http import server
from threading import Condition
import io
import os
import logging
import picamera
import socket
import socketserver

from utils import setup_logging


# Flag indicating if the robot is in the park mode in which the mobility is
# disabled. This is a safety measure to prevent us from moving the robot by
# mistake. To move the robot, we need to press "Unpark" from the web interface
# first.
unpark = False
# The IP address and port where the GoPiGo robot server is located.
gopigo_address_port = ("127.0.0.1", 8001)
# Open a UDP socket to connect to the GoPiGo server
socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# A buffer to hold a video frame. The Pi camera will write to this buffer. As a
# result, this buffer will notifythe StreamHandler to send out the new frame to
# the video streaming client.
video_stream_buffer = None
# The web page to be displayed on the browser
web_gui_page = None


def send_command_to_robot_server(command: str) -> None:
    '''Send a command to the GoPiGo robot server'''
    bytes_to_send = str.encode(command)
    socket.sendto(bytes_to_send, gopigo_address_port)


class VideoStreamBuffer(object):
    '''A class for a video buffer'''
    def __init__(self) -> None:
        '''Constructore'''
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = Condition()

    def write(self, buf) -> None:
        '''Copy the buffer content to local buffer'''
        if buf.startswith(b'\xff\xd8'):
            # A new frame becomes available. Copy the content of the buffer and
            # notify all clients so that they can refresh.
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)


class StreamingHandler(server.BaseHTTPRequestHandler):
    def load_index_page(self):
        content = web_gui_page.encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'text/html')
        self.send_header('Content-Length', len(content))
        self.end_headers()
        self.wfile.write(content)

    def redirect_to_index_page(self):
        self.send_response(301)
        self.send_header('Location', '/index.html')
        self.end_headers()

    def handle_command(self, command):
        self.send_response(200)
        self.end_headers()

        # Send command to the GoPiGo robot server
        if not unpark and command in \
                [ 'forward', 'backward', 'left', 'right', 'stop' ]:
            return
        send_command_to_robot_server(command)

    def handle_video_stream(self):
        self.send_response(200)
        self.send_header('Age', 0)
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
        self.end_headers()
        try:
            while True:
                with video_stream_buffer.condition:
                    video_stream_buffer.condition.wait()
                    frame = video_stream_buffer.frame
                self.wfile.write(b'--FRAME\r\n')
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', len(frame))
                self.end_headers()
                self.wfile.write(frame)
                self.wfile.write(b'\r\n')
        except Exception as e:
            logging.warning('Removed streaming client %s: %s',
                            self.client_address, str(e))

    def do_GET(self):
        '''Handle the HTTP get method'''
        if self.path == '/':
            self.redirect_to_index_page()
        elif self.path == '/index.html':
            self.load_index_page()
        elif self.path == '/stream.mjpg':
            self.handle_video_stream()
        elif self.path in [ '/forward', '/backward', '/left', '/right', '/stop',
                            '/pan_left', '/pan_right', '/tilt_up', '/tilt_down' ]:
            command = self.path[1:]
            self.handle_command(command)
        elif '/set_speed' in self.path:
            speed = self.path[len('/set_speed/'):]
            command = f'speed={speed}'
            self.handle_command(command)
        elif self.path == '/unpark':
            global unpark
            # Toggle unpark mode
            self.send_response(200)
            self.end_headers()
            unpark = not unpark
        else:
            # Unknown URL
            self.send_error(404)
            self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    '''A video streaming server class'''
    allow_reuse_address = True
    daemon_threads = True


def load_web_page_from_template():
    '''Load the web page from the template file'''
    web_template_filename = os.path.join(os.getcwd(), 'templates/index.html')
    with open(web_template_filename, 'r', encoding='utf-8') as index_file:
        web_gui_page = index_file.read()
    return web_gui_page


def main():
    '''The main program'''
    global web_gui_page

    setup_logging()
    logging.info('Serving http://gopigo.local:8000/index.html')

    # Load the web interface from the template file
    web_gui_page = load_web_page_from_template()

    resolution = '1440x1080'
    framerate = 30
    with picamera.PiCamera(resolution=resolution, framerate=framerate) as camera:
        global video_stream_buffer

        video_stream_buffer = VideoStreamBuffer()
        # Start the recording to the VideoStreamBuffer class who will, in turn,
        # tells the StreamingHandler to push the new frames to the video
        # streaming client.
        camera.start_recording(video_stream_buffer, format='mjpeg')
        try:
            # Start a web server to handle requests
            web_address_port = ('', 8000)
            server = StreamingServer(web_address_port, StreamingHandler)
            server.serve_forever()
        except KeyboardInterrupt:
            logging.info('Good bye!')
        finally:
            # Stop the camera
            camera.stop_recording()


if __name__ == '__main__':
    main()
