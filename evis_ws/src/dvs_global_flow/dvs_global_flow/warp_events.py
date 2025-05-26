from dvs_msgs.msg import Event
import cv2

Point2D = tuple[int, int]

def warp_events(vel: Point2D, e: Event, t_ref: float, warped_point: Point2D) -> Point2D:
    print("yipeee")