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
    def __init__(self):
        threading.Thread.__init__(self)
        # TODO: Depending on whether or not the Lidar C code can handle an
        # "ungraceful" crash, set daemon equal to True or False. True if it can
        # be terminated spontaneously without causing problems... Defaults to
        # false, I believe
        self.daemon = True
        self.lidar_queue = Queue.Queue()
        
        GPIO.setmode(GPIO.BCM)     # Number GPIOs by channelID
        GPIO.setwarnings(False)    # Ignore Errors
        
        # Setup all GPIO pins as low.
        GPIO.setup(6, GPIO.OUT)     # GPIO_06 = Lidar0
        GPIO.setup(13, GPIO.OUT)     # GPIO_13 = Lidar1
        GPIO.setup(19, GPIO.OUT)     # GPIO_19 = Lidar2
        GPIO.setup(26, GPIO.OUT)     # GPIO_26 = Lidar3
        
        self.chip_enable_lidars()
        
    def chip_enable_lidars(self):
        # Chip Enable by pulling high
        GPIO.output(6, 1)
        GPIO.output(13, 1)
        GPIO.output(19, 1)
        GPIO.output(26, 1)
        
    def setup_processes(self):
        # Create the subprocesses
        self.lidar_west_process = subprocess.Popen(
                            ['/home/pi/A-Maze/standalone/initializei2c/runme0'],
                            stdout = subprocess.PIPE)
        self.lidar_north_process = subprocess.Popen(
                            ['/home/pi/A-Maze/standalone/initializei2c/runme1'],
                            stdout = subprocess.PIPE)
        self.lidar_east_process = subprocess.Popen(
                            ['/home/pi/A-Maze/standalone/initializei2c/runme2'],
                            stdout = subprocess.PIPE)
        self.lidar_south_process = subprocess.Popen(
                            ['/home/pi/A-Maze/standalone/initializei2c/runme3'],
                             stdout = subprocess.PIPE)
                                                   
    def get_data(self):
        # Run subprocesses and get standard output. Keep track of timestamps
        # corresponding to data
        
        # This may not be the best way to do this. May need try/except to catch
        # possible exceptions
        self.setup_processes()

        north_output = self.lidar_north_process.communicate()[0]
        north_time = time.time()
        north_dist = self.get_dist_from_output(north_output)
        
        south_output = self.lidar_south_process.communicate()[0]
        south_time = time.time()
        south_dist = self.get_dist_from_output(south_output)

        east_output = self.lidar_east_process.communicate()[0]
        east_time = time.time()
        east_dist = self.get_dist_from_output(east_output)

        west_output = self.lidar_west_process.communicate()[0]
        west_time = time.time()
        west_dist = self.get_dist_from_output(west_output)
        
        lidar_data = LidarData(north_time, north_dist, south_time, south_dist,
                         east_time, east_dist, west_time, west_dist)
        
        # self.lidar_queue.put([(north_time, north_dist),
        #                       (south_time, south_dist),
        #                       (east_time, east_dist),
        #                       (west_time, west_dist)])
        
        self.lidar_queue.put(lidar_data)
        
    def get_dist_from_output(self, output):
        # Parse out the "Distance: " characters in the output string of the
        # lidar reading process. This could be changed on the C code side
        return int(output.replace('Distance: ', ''))
        
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
