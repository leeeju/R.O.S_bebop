#-*- coding: utf-8 -*-
import cv2
'''
이미지 겹치기
'''


def passFunction(x):
    pass


def main():
    windowName = "Transition Effect"

    cv2.namedWindow("Transition Effect")

    imageOne = cv2.imread("/home/kicker/Finger-Detection-and-Tracking/Sample images/lena_color_512.tif", 1) #사진1
    imageTwo = cv2.imread("/home/kicker/Finger-Detection-and-Tracking/Sample images/mandril_color.tif", 1)  #사진2

    cv2.createTrackbar("Alpha", windowName, 0, 1000, passFunction)
    # 트렉커의 시잔위치 및 최종위치
    while True:

        alpha = cv2.getTrackbarPos("Alpha", windowName) / 500   # 이미지 뱐환 기준점 (500)
        beta = 1 - alpha

        output = cv2.addWeighted(imageOne, alpha, imageTwo, beta, 0)

        cv2.imshow(windowName, output)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
