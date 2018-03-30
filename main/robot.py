#import threading
#import Queue
import time
import datetime
import pigpio
import ctypes
import os
import math

from lidar_reader import LidarReader
from servo_handler import ServoHandler

# To access Raspberry Pi's GPIO pins:
#import RPi.GPIO as GPIO

class Robot(object):

    def __init__(self):
        self.servo_handler = None
        self.initialize_lidars()
        self.initialize_servos()
        self.lidar_queue = self.lreader.lidar_queue
        # At a distance of 50 mm, robot is close to wall
        self.near_wall_thresh = 60
        self.motion_list = [self.servo_handler.move_north,
                            self.servo_handler.move_south,
                            self.servo_handler.move_east,
                            self.servo_handler.move_west]

    # TODO: theta (heading) should perhaps be accounted for. Currently set to a
    #       default value of 0.
    def move(self, u, v, t, theta=0):
        pass
        
    def initialize_lidars(self):
        # Create a LidarReader object
        self.lreader = LidarReader()
        # Call the LidarReader object's start() method, starting its thread
        self.lreader.start()
        
    def initialize_servos(self):
        self.servo_handler = ServoHandler()
    
    def mainloop(self):
        print 'Start of main loop'
        t_elapsed = 0
        t_start = time.time()

        # Get initial angular positions of each servo:
        prev_angles = self.servo_handler.get_angle_position_feedback()
        
        self.prev_move_dir = None
        
        # Cumulative displacement
        cumulative_disp = [[0, 0],
                   [0, 0],
                   [0, 0],
                   [0, 0]]
        lidar_data = None # indicates absence of lidar data at any given loop
        while True:
            # lidar_data = self.lidar_queue.get(block=True)
            # Try to read Lidar data, if new data has come in
            if not self.lidar_queue.empty():
                lidar_data = self.lidar_queue.get()
                print '------------------'
                print 'LIDAR:'
                print lidar_data.to_string()
            angles = self.servo_handler.get_angle_position_feedback()
            delta_angles = self.servo_handler.get_delta_angles(prev_angles,
                                                               angles)
            displacements = self.servo_handler.compute_linear_displacements(
                                                                delta_angles)
                                                                
            # Add displacements to cumulative displacements
            for i in range(len(displacements)):
                cumulative_disp[i][0] += displacements[i][0]
                cumulative_disp[i][1] += displacements[i][1]
                
            self.servo_handler.print_displacements(displacements)
            
            # Make the robot move based on lidar data and servo position
            # feedback
            # TODO: Implement servo position feedback
            # TODO: This is a very naive test of movement based on sensor data
            if lidar_data is not None:
                self.move_to_open_space(lidar_data) # TODO
                
            prev_angles = angles

            lidar_data = None
            
            t_elapsed = time.time() - t_start
            
    # TODO: Use a better algorithm for movement. This is just for basic testing
    # of responsiveness to lidar data
    def move_to_open_space(self, lidar_data):
        data = [lidar_data.north_dist, lidar_data.south_dist,
                lidar_data.east_dist, lidar_data.west_dist]
        room_to_move = []
        num_move_options = 0
        for dist in data:
            if dist > self.near_wall_thresh:
                room_to_move.append(True)
                num_move_options += 1
            else:
                room_to_move.append(False)
                
        if self.prev_move_dir is None:
            # Set initial motion
            self.prev_move_dir = room_to_move.index(True)
            self.move_by_lidar_wall_avoidance(self.prev_move_dir)
            return

        if num_move_options == 1:
            self.move_dir = room_to_move.index(True)
        elif num_move_options > 1:
            # There are multiple feasible directions
            # If the current direction is not a feasible direction, then go in
            # the first free one.
            if not room_to_move[self.prev_move_dir]:
                # Go in the first free one
                self.move_dir = room_to_move.index(True)
                
        if self.prev_move_dir != self.move_dir:
            # Only send a servo signal if the movement directions are different
            self.move_by_lidar_wall_avoidance(self.move_dir)
        self.prev_move_dir = self.move_dir

            
    def move_by_lidar_wall_avoidance(self, direction):
        # Based on lidar closeness
        # direction is an integer indicating which of the 4 cardinal directions
        # to move in. 0 = north, 1 = south, 2 = east, 3 = west
        self.motion_list[direction]()
        
    #def is_about_to_hit_wall(self)
        
    def shutdown(self):
        # TODO: Add things to this?
        if self.servo_handler is not None:
            self.servo_handler.close_handler()
                
        
if __name__ == '__main__':
    robot = Robot()
    try:
        robot.mainloop()
    except KeyboardInterrupt:
        robot.shutdown()
