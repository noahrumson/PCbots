import pigpio
import time

pi = pigpio.pi()

# Servo signal pins
ne = 4 # northeast
nw = 17 # northwest
sw = 22 # southwest
se = 10 # southeast

# Set RPi pins as outputs
pi.set_mode(ne, pigpio.OUTPUT)
pi.set_mode(nw, pigpio.OUTPUT)
pi.set_mode(se, pigpio.OUTPUT)
pi.set_mode(sw, pigpio.OUTPUT)

# Set PWM frequencies to 50 (shouldn't change)
pi.set_PWM_frequency(ne, 50)
pi.set_PWM_frequency(nw, 50)
pi.set_PWM_frequency(se, 50)
pi.set_PWM_frequency(sw, 50)

# Set pulsewidths, which controls each servo's velocity
# 1500 is neutral, 1300 is max clockwise, 1700 is max counterclockwise
pi.set_servo_pulsewidth(ne, 1300)
pi.set_servo_pulsewidth(nw, 1700)
pi.set_servo_pulsewidth(se, 1300)
pi.set_servo_pulsewidth(sw, 1700)
time.sleep(3)
pi.set_servo_pulsewidth(ne, 1500)
pi.set_servo_pulsewidth(nw, 1500)
pi.set_servo_pulsewidth(se, 1500)
pi.set_servo_pulsewidth(sw, 1500)
