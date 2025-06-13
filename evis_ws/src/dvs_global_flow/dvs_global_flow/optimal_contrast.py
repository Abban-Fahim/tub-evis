import numpy as np
import cv2
import scipy
import scipy.optimize

def contrast_meanSquare(img: cv2.Mat):
    contrast = cv2.norm(img, cv2.NORM_L2SQR) / (img.size[0]*img.size[1])
    return contrast

def contrast_variance():
    contrast = 0
    # TODO
    return contrast

def findFlowBruteForce(rX, rY, step):
    vel = (0, 0)
    # TODO
    
    return vel

def findInitialFlow():
    v_opt = (0, 0)
    steps = [1000., 500., 250., 125., 50., 25.]
    for i in range(1, len(steps)):
        prev = steps[i-1]
        step = steps[i]
        range_x = (v_opt[0]-prev, v_opt[0]+prev)
        range_y = (v_opt[1]-prev, v_opt[1]+prev)
        v_opt = findFlowBruteForce(range_x, range_y, (step, step))
    return v_opt

def maximizeContrast():
    
    

    # scipy.optimize.minimize(method="CG")

    sumn = 0
    return sumn
    

# Based on https://github.com/tub-rip/dvs_global_flow_skeleton/blob/master/src/numerical_deriv.cpp
def forward_diff_gradient(x: np.ndarray, func_f, data=None, dh = 1e6):
    fx = func_f(x, data)
    xh = x.copy(x)
    j = np.ndarray(x.shape)

    for i in range(len(j)):
        xh[i] = x[i] + dh
        fh = func_f(xh, data)
        j[i] = fh - fx
        xh[i] = x[i]
    # J *= 1/dh
    j /= dh

    return j, fx