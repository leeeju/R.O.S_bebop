#-*- coding: utf-8 -*-
import cv2


def main():
    image = cv2.imread("/home/kicker/Finger-Detection-and-Tracking/Sample images/house.tiff", 1)

    # Instead of split sue this way, more fast
    blue = image[:, :, 0]
    green = image[:, :, 1]
    red = image[:, :, 2]

    # 각 색상 간격을 알파 채널과 결합
    rgba = cv2.merge((blue, green, red, red))

    # cv2 gui does not support alpha value, so use default OS viewer
    # use png format, because jpeg does not support alpha channel
    print(cv2.imwrite("/home/kicker/Finger-Detection-and-Tracking/Sample images/rgba_red.png", rgba))


if __name__ == '__main__':
    main()
