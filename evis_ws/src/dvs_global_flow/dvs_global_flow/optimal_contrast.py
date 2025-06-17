import numpy as np
import cv2
import scipy
import scipy.optimize
from dvs_msgs.msg import Event
from .utils import Mat, Point2D, Size
from .warp_events import compute_image

def contrast_meanSquare(img: Mat):
    contrast = cv2.norm(img, cv2.NORM_L2SQR) / (img.shape[0]*img.shape[1])
    return contrast

def contrast_variance(img: Mat):
    contrast = 0
    # TODO
    return contrast

def compute_contrast(img: Mat, contrast_measure: int):
    if contrast_measure == 0:
        c = contrast_meanSquare(img)
    elif contrast_measure == 1:
        c = contrast_variance(img)
    
    return c

# Cost function
def contrast_f_numerical(vel: Point2D, events_subset: list[Event], img_size: Size):
    img = compute_image(vel, events_subset, img_size, False, 0)
    contrast = compute_contrast(img, 0)
    return -contrast

# Optional: Derivative of cost function
def contrast_df_numerical(vel: Point2D, events_subset: list[Event], img_size: Size):
    res, _ = forward_diff_gradient(vel, lambda x: contrast_f_numerical(x, events_subset, img_size), 1)
    # print("d[contrast]", res)
    return res

# Combined cost and derivative aren't needed in scipy
# def contrast_fdf_numerical(v: Point2D, f: float, df)

def findFlowBruteForce(rx, ry, step, events_subset: list[Event], img_size: Size):
    opt_vel = np.array([0, 0])
    min_cost = 0
    
    for vx in range(rx[0], rx[1], step[0]):
        for vy in range(ry[0], ry[1], step[1]):
            cost = contrast_f_numerical(np.array([vx,vy]), events_subset, img_size)
            if (cost < min_cost):
                min_cost = cost
                opt_vel = np.array([vx,vy])
    
    # print("Brute force vel:", opt_vel)
    return opt_vel

def findInitialFlow(events_subset: list[Event], img_size: Size):
    v_opt = np.array([0, 0])
    steps = [1000, 500, 250, 125, 50, 25]
    for i in range(1, len(steps)):
        prev = steps[i-1]
        step = steps[i]
        range_x = (v_opt[0]-prev, v_opt[0]+prev)
        range_y = (v_opt[1]-prev, v_opt[1]+prev)
        v_opt = findFlowBruteForce(range_x, range_y, (step, step), events_subset, img_size)
    return v_opt

def maximizeContrast(initial_flow: Point2D, events_subset: list[Event], img_size: Size):
    result = scipy.optimize.minimize(
        fun=lambda x: contrast_f_numerical(x, events_subset, img_size),
        jac=lambda x: contrast_df_numerical(x, events_subset, img_size),
        x0=initial_flow, 
        method="CG"
    )
    # print("optimization done: ", result)
    return result.x
    
def forward_diff_gradient(x: np.ndarray, func_f, dh = 1e6):
    fx = func_f(x)
    xh = x.copy()
    j = np.ndarray(x.shape)

    for i in range(len(j)):
        xh[i] = x[i] + dh
        fh = func_f(xh)
        j[i] = fh - fx
        xh[i] = x[i]
    # J *= 1/dh
    j /= dh

    return j, fx