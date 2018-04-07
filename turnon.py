import pigpio
import time

pi_handle = pigpio.pi()

pi_handle.write(21, 0)
pi_handle.write(20, 0)
pi_handle.write(16, 0)
pi_handle.write(12, 0)

pi_handle.write(12, 1)
time.sleep(0.1)
pi_handle.write(16, 1)
time.sleep(0.1)
pi_handle.write(20, 1)
time.sleep(0.1)
pi_handle.write(21, 1)
time.sleep(.4)
pi_handle.write(12, 0)
time.sleep(0.1)
pi_handle.write(16, 0)
time.sleep(0.1)
pi_handle.write(20, 0)
time.sleep(0.1)