import ctypes
import os

lib = ctypes.cdll.LoadLibrary(os.path.abspath('../libservofeedback.so'))
lib.init()
lib.servo_angle_ne.restype = ctypes.c_double
angle = lib.servo_angle_ne()
print angle
