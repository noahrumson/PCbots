import pigpio # The gpio library to rule them all
import threading
import Queue
import time
import subprocess

# TODO:

# TODO: Should the C implementation run in a continuous loop, printing to
# stdout, and then should this program read stdout as it comes out and before
# the program terminates, if possible? Or, should LidarReader call a C command
# that only outputs one instance of data? This distinction will impact the use
# of the subprocess, as I believe the communicate method waits until the
# subprocess terminates.

# TODO: How does LidarReader interface with the initialization of the lidar
# sensors and their address collisions?

class ButtonInput(threading.Thread):
    def __init__(self, lib, pi_handler):
        threading.Thread.__init__(self)
    	# Maps button names to their signal pins
        self.buttons = {'white': 23, 'black': 24}
        # The white button is on gpio 23
        # The white button is on gpio 24

        self.daemon = True
        # init the handler
        self.pi = pi_handler

    def buttonpress_callback(self, button, callback_func):
        gpio = self.buttons[button]
        self.pi.callback(gpio, pigpio.RISING_EDGE, callback_func)

    def run(self, button, callback_func):
    	
        self.buttonpress_callback(button, callback_func)
        while True:
            time.sleep(1)

class LEDOutput(threading.Thread):
    def __init__(self, lib, pi_handler):
        threading.Thread.__init__(self)
		# Maps LED names to their gpio pins
        self.led_names = {'green': 21, 'red1': 20, 'red2': 16, 'red3': 12}
        # # LED signal pins (in order on the board)
        # Green LED: 21
        # First Red LED: 20
        # Second Red LED: 16
        # Third Red LED: 12
        self.daemon = True

        self.pi = pi_handler

	#def blink_led

