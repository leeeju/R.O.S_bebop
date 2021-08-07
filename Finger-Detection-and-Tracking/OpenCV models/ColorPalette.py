#-*- coding: utf-8 -*-
import cv2
import numpy as np

'''
OpenCV BGR Color 이해 하기 
화면상의 색을 getTrackbarPos 를 사용하여 변경하면서 Open_CV의 B,G,R 을 이해한다 // R.B.G 와는 다르다
'''

def passFunction(x):
    pass


def main():
    windowName = "OpenCV BGR Color Palette"
    imageData = np.zeros((512, 512, 3), np.uint8)
    
    cv2.namedWindow(windowName)

    cv2.createTrackbar('Blue', windowName, 0, 255, passFunction)
    cv2.createTrackbar('Green', windowName, 0, 255, passFunction)
    cv2.createTrackbar('Red', windowName, 0, 255, passFunction)

    while (True):
        cv2.imshow(windowName, imageData)

        if cv2.waitKey(1) & 0xFF == 27:
            break

        blue = cv2.getTrackbarPos('Blue', windowName)    # 색갈 조절 버튼을 만든어 준다
        green = cv2.getTrackbarPos('Green', windowName)  # 색갈 조절 버튼을 만든어 준다
        red = cv2.getTrackbarPos('Red', windowName)      # 색갈 조절 버튼을 만든어 준다 

        imageData[:] = [blue, green, red]
        print(blue, green, red)

    cv2.destroyWindow(windowName)


if __name__ == '__main__':
    main()
