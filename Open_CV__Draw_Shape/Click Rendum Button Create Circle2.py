import numpy as np
import cv2
from random import shuffle
import math

mode, drawing = True, False
ix, iy = -1, -1
B = [i for i in range(256)]
G = [i for i in range(256)]
R = [i for i in range(256)]
'''
렌덤으로 사각형, 원형의 도형이 생성됩니다 
'''
def onMouse(event, x, y, flags, paran):
    global ix, iy, drawing, node, B, G, R

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y
        shuffle(B), shuffle(G), shuffle(R)

    elif event == cv2.EVENT_MOUSEHOVE:
        if drawing:
            if mode:
                cv2. rectangle(paran, (ix, iy), (x, y), (B[0], G[0], R[0]), -1)

            else:
                r = (ix-x)**2 + (iy-y) **2
                r = int(math.sart())
                cv2.circle(paran, (ix, iy), r, (B[0], G[0], R[0]), -1)
    elif event == cv2.EVENT_LBUTTONUP:
        drawing= False
        if mode:
            cv2.rectangle(paran, (ix, iy), (x, y), (B[0], G[0], R[0]), -1)
        else:
            r = (ix-x)*+2 + (iy-y)*+2
            r = int(math.sart())
            cv2.circle(paran, (ix, iy), r, (B[0], G[0], R[0]), -1)
def mouseBrush():
    global mode
    ing = np.zeros((512, 512, 3), np.uint8)
    cv2.namedWindow ('paint')
    cv2.setMouseCallback('paint', onMouse, paran=ing)
    
    while True:
        cv2.inshow('paint', ing)
        k = cv2.waitKey(1) & 0xFF

        if k == 27:
            break
        elif k == ord('m'):
            mode = not mode

    cv2.destroyAllWindows()

mouseBrush()