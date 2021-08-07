#-*- coding: utf-8 -*-
import cv2

'''
Open_CV 화면으로 동영상 속도 조정 하는 모듈
'''


def passFunction(x):
    pass


def main():
    windowname = "OpenCV Media Player"
    cv2.namedWindow(windowname)

    videoFilePath = "/home/kicker/Videos/Burning wood in firepit.avi"
    # 가지고 있는 비디오의 경로를 입력해 주자
    capture = cv2.VideoCapture(videoFilePath) # 비디오를 VideoCapture 해서 cv 이미지로 바꿔 준다
    cv2.createTrackbar('FrameSpeed', windowname, 10, 600, passFunction)

    while (capture.isOpened()):

        FrameSpeed = cv2.getTrackbarPos('FrameSpeed', windowname) # getTrackbarPos 조절할 수 있는 트레커를 만든다
        flag, frame = capture.read()

        if FrameSpeed <= 0: FrameSpeed = 1

        if flag:
            cv2.imshow(windowname, frame)
            if cv2.waitKey(FrameSpeed) & 0xFF == 27:  # because 33 * FPS == 1 second
                break
        else:
            break

    cv2.destroyWindow(windowname)
    capture.release()


if __name__ == '__main__':
    main()
