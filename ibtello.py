#! /usr/bin/env python3

# Ioannis Broumas
# ioabro17@student.hh.se
# Dec 2020
# Remember OpenCV is BGR

# Based on:
# https://github.com/dji-sdk/Tello-Python
# https://github.com/damiafuentes/DJITelloPy/blob/master/djitellopy/tello.py
# https://github.com/Virodroid/easyTello/blob/master/easytello/tello.py

import PIL
import socket
import threading
from threading import Thread
import time
import numpy as np
import sys

import cv2 as cv

class Tello:
    """Wrapper class to interact with the Tello drone."""

    # conversion functions for state protocol fields
    state_field_converters = {
        # Tello EDU with mission pads enabled only
        'mid': int,
        'x': int,
        'y': int,
        'z': int,
        # 'mpry': (custom format 'x,y,z')

        # common entries
        'pitch': int,
        'roll': int,
        'yaw': int,
        'vgx': int,
        'vgy': int,
        'vgz': int,
        'templ': int,
        'temph': int,
        'tof': int,
        'h': int,
        'bat': int,
        'baro': float,
        'time': int,
        'agx': float,
        'agy': float,
        'agz': float,
    }

    # cap = cv.VideoCapture("udp://@0.0.0.0:11111")
    # background_frame_read = None

    def __init__(self, local_ip, local_port, command_timeout=7, tello_ip='192.168.10.1',
                 tello_port=8889):
        """
        Binds to the local IP/port and puts the Tello into command mode.
        :param local_ip (str): Local IP address to bind.
        :param local_port (int): Local port to bind.
        :param imperial (bool): If True, speed is MPH and distance is feet.
                             If False, speed is KPH and distance is meters.
        :param command_timeout (int|float): Number of seconds to wait for a response to a command.
        :param tello_ip (str): Tello IP.
        :param tello_port (int): Tello port.
        """
        
        self.abort_flag = False
        self.command_timeout = command_timeout
        self.response = None  

        self.frame = None  # numpy array BGR -- current camera output frame
        self.last_frame = None

        # self.cap = cv2.VideoCapture("udp://@0.0.0.0:11111")

        self.tello_ip = tello_ip
        self.tello_address = (tello_ip, tello_port)
        
        # Commands
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # socket for sending cmd
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((local_ip, local_port))

        # thread for receiving cmd ack
        self.receive_thread = threading.Thread(target=self._receive_thread)
        self.receive_thread.daemon = True
        self.receive_thread.start()

        self.socket.sendto(b'command', self.tello_address)

        # Video
        # self.socket_video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # socket for receiving video stream
        # self.local_video_port = 11111  # port for receiving video stream
        # self.socket_video.bind((local_ip, self.local_video_port))

        # thread for receiving video
        # self.receive_video_thread = threading.Thread(target=self._receive_video_thread)
        # self.receive_video_thread.daemon = True
        # self.receive_video_thread.start() 

        # to receive video -- send cmd: command, streamon
        self.socket.sendto(b'streamon', self.tello_address)

        self.stream_state = True

        # TELLO STATE
        self.state = {}

        self.socket_state = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # socket for receiving state
        self.state_port = 8890  # port for receiving state
        self.socket_state.bind((local_ip, self.state_port))

        # thread for receiving state
        self.receive_state_thread = threading.Thread(target=self._receive_state_thread)
        self.receive_state_thread.daemon = True
        self.receive_state_thread.start() 

        self.socket_state.sendto('command'.encode('utf-8'), self.tello_address)


    def shutdown(self):
        """Closes the local socket."""
        self.socket.close()
        # self.socket_video.close()
        self.socket_state.close()

    def _receive_thread(self):
        """Listen to responses from the Tello.
        Runs as a thread, sets self.response to whatever the Tello last returned.
        """
        while True:
            try:
                self.response, ip = self.socket.recvfrom(2048)
                print("Response ", self.response)
            except socket.error as exc:
                print ("Receive Thread caught exception socket.error : %s" % exc)

    def read(self):
        """Return the last frame from camera."""
        frame = self.last_frame
        return frame

    # def _receive_video_thread(self):
    #     """
    #     Listens for video streaming from the Tello.
    #     Runs as a thread, sets self.frame to the most recent frame Tello captured.
    #     """
    #     cap = cv.VideoCapture("udp://@0.0.0.0:11111")
    #     print("Hej!")
    #     while self.stream_state:
    #         ret, self.frame = cap.read()
    #         if (ret):
    #             cv.imshow('Tello',self.frame)
    #         # print("Hej igen!")

    def _video_thread(self):
        # Creating stream capture object
        cap = cv.VideoCapture("udp://@0.0.0.0:11111")  # ('udp://@' + self.tello_ip + ':11111')
        # Runs while 'stream_state' is True
        print("Hej!")
        while self.stream_state:
            ret, self.last_frame = cap.read()
        #     if ret:
        #         #cv.imshow('DJI Tello', self.last_frame)

        #         # Video Stream is closed if escape key is pressed
        #         k = cv.waitKey(1) & 0xFF
        #         if k == 27:
        #             break
        # cap.release()
        # cv.destroyAllWindows()

    def stream_on(self):
        self.send_command('streamon')
        self.stream_state = True
        self.video_thread = threading.Thread(target=self._video_thread)
        self.video_thread.daemon = True
        self.video_thread.start()

    def stream_off(self):
        self.stream_state = False
        self.send_command('streamoff')

    # def stream_on(self):
    #     """Turn on video streaming. Use `tello.get_frame_read` afterwards.
    #     Video Streaming is supported on all tellos when in AP mode (i.e.
    #     when your computer is connected to Tello-XXXXXX WiFi network).
    #     Currently Tello EDUs do not support video streaming while connected
    #     to a wifi network.
    #     !!! note
    #         If the response is 'Unknown command' you have to update the Tello
    #         firmware. This can be done using the official Tello app.
    #     """
    #     self.send_command("streamon")
    #     self.stream_state = True

    # def stream_off(self):
    #     """Turn off video streaming.
    #     """
    #     self.send_command("streamoff")
    #     self.stream_state = False

    
    def parse_state(self, state: str):
        """Parse a state line to a dictionary
        Internal method, you normally wouldn't call this yourself.
        """
        state = state.strip()
        state = state.split(';')

        if len(state) < 2:
            print(state)
            return

        for field in state:
            split = field.split(':')
            if len(split) < 2:
                continue

            key = split[0]
            value = split[1]

            if key in Tello.state_field_converters:
                try:
                    value = Tello.state_field_converters[key](value)
                except Exception as e:
                    print('Error parsing state value for {}: {} to {}'
                                       .format(key, value, Tello.state_field_converters[key]))
            self.state[key] = value
        return

    def _receive_state_thread(self):
        """
        Listens for the state from the Tello.
        Runs as a thread.
        """
        while True:
            try:
                message, ip = self.socket_state.recvfrom(2048)
                # print(message.decode('utf-8'))
                self.parse_state(message.decode('ASCII')) # 'utf-8' ?
            except socket.error as exc:
                print ("State_thread caught exception socket.error : %s" % exc)

    def send_command(self, command):
        """
        Send a command to the Tello and wait for a response.
        :param command: Command to send.
        :return (str): Response from Tello.
        """
        print (">> send cmd: {}".format(command))
        self.abort_flag = False
        timer = threading.Timer(self.command_timeout, self.set_abort_flag)

        self.socket.sendto(command.encode('utf-8'), self.tello_address)

        timer.start()
        while self.response is None:
            if self.abort_flag is True:
                break
        timer.cancel()
        
        if self.response is None:
            response = 'none_response'
        else:
            response = self.response.decode('utf-8')

        self.response = None

        return response

    
    def set_abort_flag(self):
        """
        Sets self.abort_flag to True.
        Used by the timer in Tello.send_command() to indicate to that a response   
        timeout has occurred.
        """
        self.abort_flag = True

    def takeoff(self):
        """
        Initiates take-off.
        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.
        """
        return self.send_command('takeoff')

    def set_speed(self, speed):
        """
        Sets speed.
        This method expects KPH or MPH. The Tello API expects speeds from
        1 to 100 centimeters/second.
        Metric: .1 to 3.6 KPH
        Imperial: .1 to 2.2 MPH
        Args:
            speed (int|float): Speed.
        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.
        """
        speed = float(speed)
        speed = int(round(speed * 27.7778))
        return self.send_command('speed %s' % speed)

    def rotate_cw(self, degrees):
        """
        Rotates clockwise.
        Args:
            degrees (int): Degrees to rotate, 1 to 360.
        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.
        """
        return self.send_command('cw %s' % degrees)

    def rotate_ccw(self, degrees):
        """
        Rotates counter-clockwise.
        Args:
            degrees (int): Degrees to rotate, 1 to 360.
        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.
        """
        return self.send_command('ccw %s' % degrees)

    def flip(self, direction):
        """
        Flips.
        Args:
            direction (str): Direction to flip, 'l', 'r', 'f', 'b'.
        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.
        """
        return self.send_command('flip %s' % direction)

    def get_response(self):
        """
        Returns response of tello.
        Returns:
            int: response of tello.
        """
        response = self.response
        return response

    def get_yaw(self):
        """Returns yaw of tello.
        Returns:
            int: Yaw(degrees) of tello.
        """
        return self.state['yaw']

    def get_height(self):
        """Returns height(dm) of tello.
        Returns:
            int: Height(dm) of tello.
        """
        return self.state['h']

    def get_battery(self) -> int:
        """Get current battery percentage
        Returns:
            int: 0-100
        """
        return self.state['bat']

    def land(self):
        """Initiates landing.
        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.
        """
        return self.send_command('land')

    def move(self, direction, distance):
        """Moves in a direction for a distance.
        This method expects meters or feet. The Tello API expects distances
        from 20 to 500 centimeters.
        Metric: .02 to 5 meters
        Imperial: .7 to 16.4 feet
        Args:
            direction (str): Direction to move, 'forward', 'back', 'right' or 'left'.
            distance (int|float): Distance to move.
        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.
        """
        distance = float(distance)
        distance = int(round(distance*100))
        return self.send_command('%s %s' % (direction, distance))

    def move_backward(self, distance):
        """Moves backward for a distance.
        See comments for Tello.move().
        Args:
            distance (int): Distance to move.
        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.
        """
        return self.move('back', distance)

    def move_down(self, distance):
        """Moves down for a distance.
        See comments for Tello.move().
        Args:
            distance (int): Distance to move.
        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.
        """
        return self.move('down', distance)

    def move_forward(self, distance):
        """Moves forward for a distance.
        See comments for Tello.move().
        Args:
            distance (int): Distance to move.
        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.
        """
        return self.move('forward', distance)

    def move_left(self, distance):
        """Moves left for a distance.
        See comments for Tello.move().
        Args:
            distance (int): Distance to move.
        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.
        """
        return self.move('left', distance)

    def move_right(self, distance):
        """Moves right for a distance.
        See comments for Tello.move().
        Args:
            distance (int): Distance to move.
        """
        return self.move('right', distance)

    def move_up(self, distance):
        """Moves up for a distance.
        See comments for Tello.move().
        Args:
            distance (int): Distance to move.
        Returns:
            str: Response from Tello, 'OK' or 'FALSE'.
        """
        return self.move('up', distance)

    # def get_frame_read(self) -> 'BackgroundFrameRead':
    #     """Get the BackgroundFrameRead object from the camera drone. Then, you just need to call
    #     backgroundFrameRead.frame to get the actual frame received by the drone.
    #     Returns:
    #         BackgroundFrameRead
    #     """
    #     if self.background_frame_read is None:
    #         self.background_frame_read = BackgroundFrameRead(self).start()
    #     return self.background_frame_read

# class BackgroundFrameRead:
#     """
#     This class read frames from a VideoCapture in background. Use
#     backgroundFrameRead.frame to get the current frame.
#     """

#     def __init__(self, tello):
#         tello.cap = cv.VideoCapture("udp://@0.0.0.0:11111")
#         self.cap = tello.cap

#         if not self.cap.isOpened():
#             self.cap.open("udp://@0.0.0.0:11111")

#         self.grabbed, self.frame = self.cap.read()
#         self.stopped = False

#     def start(self):
#         Thread(target=self.update_frame, args=(), daemon=True).start()
#         return self

#     def update_frame(self):
#         while not self.stopped:
#             if not self.grabbed or not self.cap.isOpened():
#                 self.stop()
#             else:
#                 (self.grabbed, self.frame) = self.cap.read()

#     def stop(self):
#         self.stopped = True