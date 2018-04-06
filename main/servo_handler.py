#import threading
import Queue
import datetime
import pigpio # GPIO library to interface with servos through the Pi's GPIO pins
import time
import ctypes # To be able to call C libraries from Python
import os
import math

# TODO: Decide whether or not we want to run a separate thread to collect
# position feedback data

# Class that handles the commanding of the servos and the reception of position
# feedback data
#class ServoHandler(threading.Thread):

class ServoHandler():

    DIRECTION_NORTH = 0
    DIRECTION_WEST = 1
    DIRECTION_SOUTH = 2
    DIRECTION_EAST = 3

    def __init__(self, lib, pi_handler):
        #threading.Thread.__init__(self)
        #self.daemon = True
        #self.servo_queue = Queue.Queue()
        
        # Maps servo names to their signal pins
        self.servo_names = {'ne': 4, 'nw': 17, 'se': 10, 'sw': 22}
        self.servo_speeds = {}
        self.direction = ServoHandler.DIRECTION_NORTH
        # # Servo signal pins
        # self.ne = 4 # northeast
        # self.nw = 17 # northwest
        # self.sw = 22 # southwest
        # self.se = 10 # southeast
        
        self.lib = lib
        
        self.pi = pi_handler
        # TODO: Every now and then there is an error with pigpio containing the
        # following. The error occurs seemingly spontaneously...:
        # "initInitialise: bind to port 8888 failed (Address already in use)"
        # "Can't connect to pigpio at localhost(8888)"

        # Set PWM frequencies to 50 (shouldn't change)
        self.pi.set_PWM_frequency(self.servo_names['ne'], 50)
        self.pi.set_PWM_frequency(self.servo_names['nw'], 50)
        self.pi.set_PWM_frequency(self.servo_names['se'], 50)
        self.pi.set_PWM_frequency(self.servo_names['sw'], 50)
        
    def send_signal(self, servo, speed):
        '''
        - servo is a string corresponding to a servo name
        - speed is a value between -200 and 200 that corresponds to angular
          velocity of servo. -200 is max speed clockwise, 0 is neutral/stop, and
          200 is max speed counterclockwise
        '''
        self.check_servo_name(servo)
        self.servo_speeds[servo] = speed
        
        # Check legality of input speed argument
        if speed < -200 or speed > 200:
            raise Exception('Illegal input argument. Speed must be a number '
                'between -200 and 200, inclusive.')
                
        # Convert speed input to pulsewidth
        pulse_width = int(speed + 1500)
        self.pi.set_servo_pulsewidth(self.servo_names[servo], pulse_width)

    def adjust_signal(self, servo, offset):
        self.check_servo_name(servo)
        self.send_signal(servo, self.servo_speeds[servo] + offset)
        
    def stop(self, servo):
        self.send_signal(servo, 0)
        
    def stop_all(self):
        for servo in self.servo_names.keys():
            self.send_signal(servo, 0)
        
    def check_servo_name(self, servo):
        if servo not in self.servo_names.keys():
            raise Exception('Illegal input argument. Servo name must be a '
                'string that is either "ne", "nw", "se", or "sw".')
        
    def close_handler(self):
        self.stop_all()
        self.pi.stop()
        self.lib.terminate()
        
    def get_angle_position_feedback(self):
        return [self.lib.servo_angle_ne(),
                self.lib.servo_angle_nw(),
                self.lib.servo_angle_se(),
                self.lib.servo_angle_sw()]
                
    def get_delta_angles(self, prev_angles, angles):
        delta_angles = []
        # Compute difference in revolutions:
        for i in range(len(angles)):
            cur_angle = angles[i]
            prev_angle = prev_angles[i]
            delta_angle = cur_angle - prev_angle

            # Check if the feedback values looped (this could perhaps be
            # improved to be more reliable. Could perhap look at servo drive
            # commands, or could look at magnitude of the difference between
            # prev and current angles, for example)
            if (prev_angle > 0.9) and (cur_angle < 0.1) and (delta_angle < 0):
                # Wheel is estimated to be going clockwise, crossed over a loop
                delta_angle = cur_angle + (1 - prev_angle)
            elif ((prev_angle < 0.1) and (cur_angle > 0.9) and
                 (delta_angle > 0)):
                # Wheel is estimated to be going counterclockwise, crossed over
                # a loop
                delta_angle = -(prev_angle + (1 - cur_angle))
            delta_angles.append(delta_angle)
        return delta_angles
        
    def move_north(self):
        # Uncorrected northward movement, medium speed
        self.direction = ServoHandler.DIRECTION_NORTH
        self.send_signal('ne', -75)
        self.send_signal('nw', 75)
        self.send_signal('se', -75)
        self.send_signal('sw', 65)
        
    def move_south(self):
        # Uncorrected southward movement, medium speed
        self.direction = ServoHandler.DIRECTION_SOUTH
        self.send_signal('ne', 60)
        self.send_signal('nw', -75)
        self.send_signal('se', 75)
        self.send_signal('sw', -75)
        
    def move_east(self):
        # Uncorrected eastward movement, medium speed
        self.direction = ServoHandler.DIRECTION_EAST
        self.send_signal('ne', 75)
        self.send_signal('nw', 60)
        self.send_signal('se', -75)
        self.send_signal('sw', -75)
        
    def move_west(self):
        # Uncorrected westward movement, medium speed
        self.direction = ServoHandler.DIRECTION_WEST
        self.send_signal('ne', -75)
        self.send_signal('nw', -75)
    #    self.send_signal('se', 65)
     #   self.send_signal('sw', 75)
        self.send_signal('se', 75)
        self.send_signal('sw', 60)

    def move_direction(self, dir):
        self.direction = dir
        if dir == ServoHandler.DIRECTION_NORTH:
            self.send_signal('ne', -75)
            self.send_signal('nw', 75)
            self.send_signal('se', -75)
            self.send_signal('sw', 65)
        elif dir == ServoHandler.DIRECTION_WEST:
            self.send_signal('ne', -75)
            self.send_signal('nw', -75)
            self.send_signal('se', 75)
            self.send_signal('sw', 60)
        elif dir == ServoHandler.DIRECTION_SOUTH:
            self.send_signal('ne', 60)
            self.send_signal('nw', -75)
            self.send_signal('se', 75)
            self.send_signal('sw', -75)
        elif dir == ServoHandler.DIRECTION_EAST:
            self.send_signal('ne', 75)
            self.send_signal('nw', 60)
            self.send_signal('se', -75)
            self.send_signal('sw', -75)

            
    # def run(self):
    #     # TODO: Run the thread
    #     pass
        
