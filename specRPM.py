import pigpio
import time

pi = pigpio.pi()

pi.set_mode(4, pigpio.OUTPUT);
pi.set_mode(17, pigpio.OUTPUT);
pi.set_PWM_dutycycle(4, 7);
pi.set_PWM_dutycycle(17, 7);
pi.set_PWM_frequency(4, 50);
pi.set_PWM_frequency(17, 50);
time.sleep(3)
pi.set_PWM_dutycycle(4, 0);
pi.set_PWM_dutycycle(17, 0);