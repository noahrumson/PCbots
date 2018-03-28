import pigpio
import time
import ctypes
import os

lib = ctypes.cdll.LoadLibrary(os.path.abspath('../libservofeedback.so'))
lib.init()
lib.servo_angle_ne.restype = ctypes.c_double
lib.servo_angle_nw.restype = ctypes.c_double
lib.servo_angle_se.restype = ctypes.c_double
lib.servo_angle_sw.restype = ctypes.c_double
pi = pigpio.pi()

# Servo signal pins
ne = 4 # northeast
nw = 17 # northwest
sw = 22 # southwest
se = 10 # southeast

# Set PWM frequencies to 50 (shouldn't change)
pi.set_PWM_frequency(ne, 50)
pi.set_PWM_frequency(nw, 50)
pi.set_PWM_frequency(se, 50)
pi.set_PWM_frequency(sw, 50)

# Set pulsewidths, which controls each servo's velocity
# 1500 is neutral, 1300 is max clockwise, 1700 is max counterclockwise
#pi.set_servo_pulsewidth(ne, 1300)
#pi.set_servo_pulsewidth(nw, 1700)
#pi.set_servo_pulsewidth(se, 1300)
pi.set_servo_pulsewidth(sw, 1700)
t_elapsed = 0
t_start = time.time()
prev_angle = None
while (t_elapsed < 3):
    ne_angle = lib.servo_angle_ne()
    nw_angle = lib.servo_angle_nw()
    se_angle = lib.servo_angle_se()
    sw_angle = lib.servo_angle_sw()
    print 'SW: ' + str(sw_angle)
    print 'SE: ' + str(se_angle)
    print 'NE: ' + str(ne_angle)
    print 'NW: ' + str(nw_angle)
    time.sleep(0.1)
    t_elapsed = time.time() - t_start

pi.set_servo_pulsewidth(ne, 1500)
pi.set_servo_pulsewidth(nw, 1500)
pi.set_servo_pulsewidth(se, 1500)
pi.set_servo_pulsewidth(sw, 1500)
pi.stop()
#lib.terminate()
