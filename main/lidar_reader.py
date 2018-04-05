import RPi.GPIO as GPIO
import threading
import Queue
import time
import subprocess

# TODO: Decide if LidarReader should handle all of the lidar data or if each
# Lidar should have its own LidarReader object, perhaps? Depends on the C
# implementation, probably

# TODO: Should the C implementation run in a continuous loop, printing to
# stdout, and then should this program read stdout as it comes out and before
# the program terminates, if possible? Or, should LidarReader call a C command
# that only outputs one instance of data? This distinction will impact the use
# of the subprocess, as I believe the communicate method waits until the
# subprocess terminates.

# TODO: How does LidarReader interface with the initialization of the lidar
# sensors and their address collisions?

class LidarReader(threading.Thread):
    def __init__(self, lib):
        
        self.lib = lib
        
        threading.Thread.__init__(self)
        # TODO: Depending on whether or not the Lidar C code can handle an
        # "ungraceful" crash, set daemon equal to True or False. True if it can
        # be terminated spontaneously without causing problems... Defaults to
        # false, I believe
        self.daemon = True
        self.lidar_queue = Queue.Queue()
        
        # GPIO.setmode(GPIO.BCM)     # Number GPIOs by channelID
        # #GPIO.setwarnings(False)    # Ignore Errors
        #
        # # Setup all GPIO pins as low.
        # GPIO.setup(6, GPIO.OUT)     # GPIO_06 = Lidar0
        # GPIO.setup(13, GPIO.OUT)     # GPIO_13 = Lidar1
        # GPIO.setup(19, GPIO.OUT)     # GPIO_19 = Lidar2
        # GPIO.setup(26, GPIO.OUT)     # GPIO_26 = Lidar3
        #
        # self.chip_enable_lidars()
        
    # def chip_enable_lidars(self):
    #     # Chip Enable by pulling high
    #     GPIO.output(6, 1)
    #     GPIO.output(13, 1)
    #     GPIO.output(19, 1)
    #     GPIO.output(26, 1)
                                                   
    def get_data(self):
        print 'GETTING LIDAR DATA'
        # Keep track of timestamps corresponding to data
        

        north_dist = self.lib.lidar_distance_north()
        north_time = time.time()
        
        south_dist = self.lib.lidar_distance_south()
        south_time = time.time()

        east_dist = self.lib.lidar_distance_east()
        east_time = time.time()

        west_dist = self.lib.lidar_distance_west()
        west_time = time.time()
        
        lidar_data = LidarData(north_time, north_dist, south_time, south_dist,
                         east_time, east_dist, west_time, west_dist)
        
        # self.lidar_queue.put([(north_time, north_dist),
        #                       (south_time, south_dist),
        #                       (east_time, east_dist),
        #                       (west_time, west_dist)])
        print 'DONE GETTING LIDAR DATA'
        self.lidar_queue.put(lidar_data)
        
    def run(self):
        # Run the thread
        while True:
            self.get_data()

# Class that stores an instance of lidar data
class LidarData(object):
    def __init__(self, north_time, north_dist, south_time, south_dist,
                       east_time, east_dist, west_time, west_dist):
        self.north_time = north_time
        self.north_dist = north_dist
        self.south_time = south_time
        self.south_dist = south_dist
        self.east_time = east_time
        self.east_dist = east_dist
        self.west_time = west_time
        self.west_dist = west_dist
        
    def to_string(self):
        return_format = '-----\n{dir}\n\tDistance (mm): {dist}\n\tTime (s): {t}\n'
        return_string = return_format.format(dir='North', dist=self.north_dist,
                                             t=self.north_time)
        return_string += return_format.format(dir='South', dist=self.south_dist,
                                             t=self.south_time)
        return_string += return_format.format(dir='East', dist=self.east_dist,
                                             t=self.east_time)
        return_string += return_format.format(dir='West', dist=self.west_dist,
                                             t=self.west_time)
        return return_string
                                             

# This should not be used in implementation, as lidar_reader.py is meant to be
# imported by robot.py so that the Robot class can use a LidarReader() object
if __name__ == '__main__':
    LidarReader().start()
