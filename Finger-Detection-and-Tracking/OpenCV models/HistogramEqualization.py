#-*- coding: utf-8 -*-
import cv2
import matplotlib.pyplot as plt
'''
히스토그램 구하기
히스토그램(histogram)은 표로 되어 있는 도수 분포를 정보 그림으로 나타낸 것이다. 
더 간단하게 말하면, 도수분포표를 그래프로 나타낸 것이다. 보통 히스토그램에서는 
가로축이 계급, 세로축이 도수를 뜻하는데, 때때로 반대로 그리기도 한다.
'''

def main():
    image = cv2.imread("/home/kicker/Finger-Detection-and-Tracking/Sample images/4.1.06.tiff", 1)  #이미지 에서 히스토 그램을 구한다
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    red, green, blue = cv2.split(image_rgb)

    eq_red_image = cv2.equalizeHist(red)
    eq_green_image = cv2.equalizeHist(green)
    eq_blue_image = cv2.equalizeHist(blue)

    red_hist = cv2.calcHist([red], [0], None, [256], [0, 255])
    green_hist = cv2.calcHist([green], [0], None, [256], [0, 255])
    blue_hist = cv2.calcHist([blue], [0], None, [256], [0, 255])

    eq_red_hist = cv2.calcHist([eq_red_image], [0], None, [256], [0, 255])
    eq_green_hist = cv2.calcHist([eq_green_image], [0], None, [256], [0, 255])
    eq_blue_hist = cv2.calcHist([eq_blue_image], [0], None, [256], [0, 255])

    channels_images = [red_hist, green_hist, blue_hist]
    equalized_images = [eq_red_hist, eq_green_hist, eq_blue_hist]

    # Channels Histogram
    for i in range(3):
        plt.subplot(4, 1, i + 1)
        plt.plot(channels_images[i], color='g')
        plt.xlim([0, 255])

    plt.show()

    # Channels Equalized Histogram
    for i in range(3):
        plt.subplot(3, 1, i + 1)
        plt.plot(equalized_images[i], color='b')
        plt.xlim([0, 255])

    plt.show()


if __name__ == '__main__':
    main()
