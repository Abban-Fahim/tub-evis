# Just some code to experiment with convolutions in Python before implemeting them properly
import numpy as np
import cv2

a = np.zeros((6,6))
# a = np.ones((3,3))
print(a)

a[2,2] = -1
print(a)

# filter = np.array([0, 0, 0, 0, 1, 0, 0, 0, 0], dtype=np.float32).reshape((3,3)) # unit
filter = np.array([-1, 0, 1, -2, 0, 2, -1, 0, 1], dtype=np.float32).reshape((3,3)) # sobel-x
# filter = 1/9*np.ones((3,3), dtype=np.float32).reshape((3,3)) # blur
print(filter)
b = cv2.filter2D(a, -1, filter)
print(b)

# c = a[2-1:2-1+3,2-1:2-1+3]
c = a[2,2]
print(c)
d = cv2.filter2D(c,-1,filter)
print(d)
