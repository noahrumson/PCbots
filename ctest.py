import ctypes
import os

lib = ctypes.cdll.LoadLibrary(os.path.abspath("librpmtest.so"))
lib.run.restype = ctypes.c_double
angle = lib.run()
print "Python received revolutions: " + str(angle)
