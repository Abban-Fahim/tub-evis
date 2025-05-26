import cv2

def contrast_meanSquare(img: cv2.Mat):
    contrast = cv2.norm(img, cv2.NORM_L2SQR) / (img.size[0]*img.size[1])
    return contrast

def contrast_variance():
    contrast = 0
    # TODO
    return contrast

def findFlowBruteForce():
    vel = (0, 0)
    # TODO
    return vel

def findInitialFlow():
    vel = (0, 0)
    # TODO
    return vel

def maximizeContrast():
    sumn = 0
    return sumn