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
    def __init__(self, lib, pi_handler, button, killqueue):
        threading.Thread.__init__(self)
    	# Maps button names to their signal pins
        self.buttons = {'white': 23, 'black': 24}
        # The white button is on gpio 23
        # The white button is on gpio 24
        self.current_button = self.buttons[button]

        self.daemon = True
        # init the handler
        self.pi = pi_handler

        self.button_queue = Queue.Queue()
        self.killqueue = killqueue

    def buttonpress_callback(self, button, callback_func):
        gpio = self.buttons[button]
        self.pi.callback(gpio, pigpio.RISING_EDGE, callback_func)

    def wait_until_press(self, button):
        gpio = self.buttons[button]
        self.pi.wait_for_edge(gpio, pigpio.RISING_EDGE, 10800) # Timeout is 3 hours

    def run(self):
    	
        while True:
            if not self.killqueue.empty():
                # End this thread
                return
            try:
                if self.pi.wait_for_edge(self.current_button, pigpio.FALLING_EDGE, 4):
                    self.button_queue.put(True)
            except AttributeError:
                # This seems like bad coding practice, but is an attempt at a quick fix
                return

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

    def wink_led(self, name):
        self.pi.write(self.led_names[name], 1)
        time.sleep(0.01)
        self.pi.write(self.led_names[name], 0)

    def led_on(self, name):
        self.pi.write(self.led_names[name], 1)

    def led_off(self, name):
        self.pi.write(self.led_names[name], 0 )
