import pigpio
import time

pi_handle = pigpio.pi()

# pi_handle.set_mode(23, pigpio.INPUT)
# pi_handle.set_mode(24, pigpio.INPUT)

pi_handle.write(12, 0)
pi_handle.write(16, 0)
pi_handle.write(20, 0)
pi_handle.write(21, 0)

def handlepress(gpio, level, tick):
	print("holy shit!", gpio, level, tick)
	if(gpio == 23):
		pi_handle.write(20, 1)
		time.sleep(.01)
		pi_handle.write(20, 0)
	elif(gpio == 24):
		pi_handle.write(21, 1)
		time.sleep(.01)
		pi_handle.write(21, 0)

pi_handle.callback(23, pigpio.RISING_EDGE, handlepress)
pi_handle.callback(24, pigpio.RISING_EDGE, handlepress)

# while True:
# 	print(pi_handle.read(23))

#time.sleep(10)

#pi_handle.stop()
