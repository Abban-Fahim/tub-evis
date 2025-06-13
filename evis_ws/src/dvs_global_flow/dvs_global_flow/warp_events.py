from dvs_msgs.msg import Event
import cv2
import numpy as np
from .utils import Point2D, Mat, Size


def warp_event(vel: Point2D, e: Event, t_ref: float) -> Point2D:
    point = [e.x, e.y]
    warped_point = point - (e.ts - t_ref) * vel
    return warped_point


def accumalate_warped_event(e: Event, img_width: int, img_height: int, warped_point: Point2D, image_warped: Mat, use_polarity: bool) -> Mat:
    p = (1 if e.polarity else -1) if use_polarity else 1
    xx = warped_point[0]
    yy = warped_point[1]

    # Ensure the voting is only done in bounds
    if ((xx >= 1 and xx < img_width - 2) and (yy >= 1 and yy < img_height - 2)):
        # Find distances for bilinear voting
        nearest_bin = np.floor(warped_point)
        x_i = nearest_bin[0] - 1 if (xx - nearest_bin[0]) <= 0.5 else nearest_bin[0]
        y_i = nearest_bin[1] - 1 if (yy - nearest_bin[1]) <= 0.5 else nearest_bin[1]
        dx = xx - (x_i + 0.5)
        dy = yy - (y_i + 0.5)
        
        image_warped[y_i,x_i] += p * (1 - dx) * (1 - dy)
        image_warped[y_i,x_i+1] += p * dx * (1 - dy)
        image_warped[y_i+1,x_i] += p * (1 - dx) * dy
        image_warped[y_i+1,x_i+1] += p * dx * dy

    return image_warped



def compute_image(vel: Point2D, events_subset: list[Event], img_size: Size, image_warped: Mat, use_polarity: bool, blur_sigma: float) -> Mat:
    # image_warped = cv2.Mat

    t_ref = events_subset[0].ts.sec + events_subset[0].ts.nanosec * 1e-9
    for e in events_subset:
        warped_event = warp_event(vel, e, t_ref)
        image_warped = accumalate_warped_event(e, img_size[1], img_size[0], warped_event, image_warped, use_polarity)
    
    if blur_sigma > 0:
        image_warped = cv2.GaussianBlur(image_warped, (3,3), blur_sigma)

    return image_warped