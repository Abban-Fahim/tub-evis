from dvs_msgs.msg import Event, EventArray
import cv2
import numpy as np
import numpy.typing as npt

Point2D = npt.ArrayLike
Mat = cv2.typing.MatLike
Size = cv2.typing.Size

def concat_horizontal(A: Mat, B: Mat) -> Mat:
    assert A.shape[0] == B.shape[0]
    assert A.dtype == B.dtype

    return np.concatenate((A, B), axis=1)