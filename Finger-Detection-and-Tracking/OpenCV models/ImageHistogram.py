#-*- coding: utf-8 -*-
import cv2
import matplotlib.pyplot as plt
import numpy as np

'''
Matplotlib를 사용한 히스토그램(그래프)
색상의 그래프화
'''

def main():
    image = cv2.imread("/home/kicker/Finger-Detection-and-Tracking/Sample images/4.1.03.tiff", 1)
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    red_hist = cv2.calcHist([image_rgb], [0], None, [256], [0, 255])
    green_hist = cv2.calcHist([image_rgb], [1], None, [256], [0, 255])
    blue_hist = cv2.calcHist([image_rgb], [2], None, [256], [0, 255])

    # Matplotlib를 사용한 히스토그램
    plt.subplot(3, 1, 1)
    plt.hist(image.ravel(), 256, [0, 255])
    plt.xlim([0, 255])
    plt.title("Image Histogram using Matplotlib")

    # Numpy를 사용한 히스토그램
    plt.subplot(3, 1, 2)
    histogram, _ = np.histogram(image.ravel(), 256, [0, 255])
    plt.plot(histogram, color='r')
    plt.xlim([0, 255])
    plt.title("Image Histogram using Numpy")

    # Numpy를 사용한 히스토그램
    plt.subplot(3, 1, 3)
    plt.plot(red_hist, color='r')
    plt.xlim([0, 255])
    plt.title("Image Histogram using OpenCV")

    plt.show()


if __name__ == '__main__':
    main()
