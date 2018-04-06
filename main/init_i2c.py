import RPi.GPIO as GPIO  
import subprocess 
from time import sleep     # this lets us have a time delay: sleep(0.1)  

GPIO.setmode(GPIO.BCM)     # Number GPIOs by channelID   
# GPIO.setwarnings(False)    # Ignore Errors

# Setup all GPIO pins as low.
GPIO.setup(6, GPIO.OUT)     # GPIO_06 = Lidar0
GPIO.setup(13, GPIO.OUT)     # GPIO_13 = Lidar1
GPIO.setup(19, GPIO.OUT)     # GPIO_19 = Lidar2
GPIO.setup(26, GPIO.OUT)     # GPIO_26 = Lidar3

GPIO.output(6, 0) # Chip Disable by pulling low
GPIO.output(13, 0) # Chip Disable by pulling low
GPIO.output(19, 0) # Chip Disable by pulling low
GPIO.output(26, 0) # Chip Disable by pulling low

try: 
    print("CE Lidar0")

    GPIO.output(6, 1) # Chip Enable by pulling high
    
    lidar0_process = subprocess.Popen(['./runme0'], stdout = subprocess.PIPE) 
    std_output = lidar0_process.communicate()[0] 
    
    print("Output: " + std_output) # Print Output

    print("CE Lidar1")

    GPIO.output(13, 1) # Chip Enable by pulling high
    
    lidar1_process = subprocess.Popen(['./runme1'], stdout = subprocess.PIPE) 
    std_output = lidar1_process.communicate()[0] 
    
    print("Output: " + std_output) # Print Output

    print("CE Lidar2")

    GPIO.output(19, 1) # Chip Enable by pulling high
    
    lidar2_process = subprocess.Popen(['./runme2'], stdout = subprocess.PIPE) 
    std_output = lidar2_process.communicate()[0] 
    
    print("Output: " + std_output) # Print Output
    
    print("CE Lidar3")

    GPIO.output(26, 1) # Chip Enable by pulling high
    
    lidar3_process = subprocess.Popen(['./runme3'], stdout = subprocess.PIPE) 
    std_output = lidar3_process.communicate()[0] 
    
    print("Output: " + std_output) # Print Output

finally: # If Ctr+C is pressed

    print("Exiting")
