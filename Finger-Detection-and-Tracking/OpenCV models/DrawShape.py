#-*- coding: utf-8 -*-
import cv2
import numpy as np

'''
마우스(드레그)를 사용한 cv 이미지 안에 도형(네모) 그리기
마우스 우클릭시 확대,축소 기능 추가.
'''
mode = True
xi, yi = -1, -1  # type: (int, int)
drawing = False

windowName = "Drawing Shapes"
image = np.zeros((512, 800, 3), np.uint8) # 색을 결정한다.
cv2.namedWindow(windowName)


def drawShape(event, x, y, flags, params): #화면의 어딘가에 도형이 그려질지 모르기 때문에 반복문을 사용하여 입력을 대기 한다.
    global mode, drawing, xi, yi

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        xi, yi = x, y

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == True:
            if mode == True:
                cv2.rectangle(image, (xi, yi), (x, y), (0, 255, 0), -1)
            else:
                cv2.circle(image, (x, y), 5, (255, 0, 0), -1)

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        if mode == True:
            cv2.rectangle(image, (xi, yi), (x, y), (0, 255, 0), -1)
        else:
            cv2.circle(image, (x, y), 5, (255, 0, 0), -1)


cv2.setMouseCallback(windowName, drawShape)


def main():
    global mode

    while True:
        preseedKey = cv2.waitKey(1)
        cv2.imshow(windowName, image)

        if preseedKey & 0xFF == ord('m') or preseedKey & 0xFF == ord('M'):
            mode = not mode
        elif preseedKey & 0xFF == 27:
            break

    cv2.destroyWindow(windowName)


if __name__ == '__main__':
    main()
