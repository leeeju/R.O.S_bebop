#-*- coding: utf-8 -*-
import cv2
import numpy as np

'''
가우시안 블러       = 흐림효과
다이엘(dialate)    = 팽창효과
이어로우션(erosion) = 침식효과
'''

def main():
    kernal = np.ones((5, 5), np.uint8)
    # kernal == 통과한다
    image = cv2.imread("/home/kicker/Finger-Detection-and-Tracking/Sample images/4.2.05.tiff", 1) # 이미지를 불러옴
                        # 경로설정 까먹지 마세요.                               # 이미지는 아무거나 상관 없음

    blurimage = cv2.GaussianBlur(image, (15, 15), 0)   # 흐림효과
    dialate = cv2.dilate(image, kernal, iterations=1)  # 팽창효과
    erosion = cv2.erode(image, kernal, iterations=1)   # 침식효과
                                      #iterations 반복  ex = 반복은 한번만 한다
    cv2.imshow("Dialated Image", dialate)
    cv2.imshow("Eroded Image", erosion)
    cv2.imshow("Blur Image", blurimage)
    cv2.imshow("Orignal Image", image)  # 원본이미지 

    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
