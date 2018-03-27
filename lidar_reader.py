import threading
import Queue
import datetime
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
        
    def run(self):
        # Run the thread
        self.run_subprocess()
        
    def run_subprocess(self):
        # TODO: Change this to the actual command that we want to call, i.e.,
        # execute the C program
        
        # Create a subprocess
        # TODO: Replace this testing example
        # lidar_subprocess = subprocess.Popen(['echo', 'testing'],
        #                     stdout=subprocess.PIPE)
                            
        # TODO: Replace this testing example
        lidar_subprocess = subprocess.Popen(['sleep', '10'],
                            stdout=subprocess.PIPE)
                            
        # Let subprocess execute/terminate, and get standard output
        std_output = lidar_subprocess.communicate()[0]
        # Keep track of timestamp of data, more or less real-time, hopefully...
        time_of_stdout = datetime.datetime.utcnow()
        processed_output = self.process_output(std_output)
        self.lidar_queue.put([time_of_stdout, processed_output])
        
        print std_output
        
    def process_output(self, std_output):
        # TODO: Process the standard output to get nicely formatted data
        return std_output
        
        
        

# This should not be used in implementation, as lidar_reader.py is meant to be
# imported by robot.py so that the Robot class can use LidarReader() objects,
# one for each lidar sensor.
if __name__ == '__main__':
    LidarReader().start()