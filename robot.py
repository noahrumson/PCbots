#import threading
#import Queue
import time
import datetime

from lidar_reader import LidarReader

# To access Raspberry Pi's GPIO pins:
#import RPi.GPIO as GPIO

class Robot(object):

    def __init__(self):
        self.initialize_lidars()
        self.lidar_queue = self.lreader.lidar_queue

    # TODO: theta (heading) should perhaps be accounted for. Currently set to a
    #       default value of 0.
    def move(self, u, v, t, theta=0):
        pass
        
    def initialize_lidars(self):
        # TODO: Should a LidarReader object be created here for each of the
        # lidar sensors? Or are we only going to use a single LidarReader?
        print 'Creating LidarReader object...'
        self.lreader = LidarReader()
        print 'Calling the LidarReader object\'s start() method, starting its thread...'
        self.lreader.start()
        print 'This is after the start of the LidarReader\'s thread'
        
    # def get_lidar_data(self):
    
    def mainloop(self):
        print 'Start of mainloop'
        while True:
            if not self.lidar_queue.empty():
                lidar_data = self.lidar_queue.get()
                print lidar_data
        
        
if __name__ == '__main__':
    robot = Robot()
    robot.mainloop()
        
    
    