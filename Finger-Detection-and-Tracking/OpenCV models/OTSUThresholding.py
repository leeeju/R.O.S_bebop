#-*- coding: utf-8 -*-
import cv2
import matplotlib.pyplot as plt
'''
-이미지 임계처리-
이미지 Thresholding는 문턱 값 이상이면 어떤 값으로 바꾸어주고 낮으면 0으로 바꾸어주는 기능을 합니다. 
예전에 전자전기개론때 배운 문턱전압이랑 비슷한 이론인 것 같습니다. 

cv2.threshold(img, threshold_value, value, flag)
img:grayScale이고 threshold_value는 픽셀 문턱값이고 문턱값 이상이면 value로 바꾸어줍니다. 


flag에서도 다양한 종류가 존재합니다. 

cv2.THRESH_BINARY: threshold보다 크면 value이고 아니면 0으로 바꾸어 줍니다. 

cv2.THRESH_BINARY_INV: threshold보다 크면 0이고 아니면 value로 바꾸어 줍니다.   

cv2.THRESH_TRUNC: threshold보다 크면 value로 지정하고 작으면 기존의 값 그대로 사용한다. 

cv2.THRESH_TOZERO: treshold_value보다 크면 픽셀 값 그대로 작으면 0으로 할당한다. 

cv2.THRESH_TOZERO_INV: threshold_value보다 크면 0으로 작으면 그대로 할당해준다. 

'''

def main():
    threshold = 0
    max_value = 255

    image = cv2.imread("/home/kicker/Finger-Detection-and-Tracking/Sample images/7.1.08.tiff", 0)

    # when applying OTSU threshold, set threshold to 0.

    _, output1 = cv2.threshold(image, threshold, max_value, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    _, output2 = cv2.threshold(image, threshold, max_value, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    _, output3 = cv2.threshold(image, threshold, max_value, cv2.THRESH_TOZERO + cv2.THRESH_OTSU)
    _, output4 = cv2.threshold(image, threshold, max_value, cv2.THRESH_TOZERO_INV + cv2.THRESH_OTSU)
    _, output5 = cv2.threshold(image, threshold, max_value, cv2.THRESH_TRUNC + cv2.THRESH_OTSU)

    images = [image, output1, output2, output3, output4, output5]
    titles = ["Orignals", "Binary", "Binary Inverse", "TOZERO", "TOZERO INV", "TRUNC"]

    for i in range(6):
        plt.subplot(3, 2, i + 1)
        plt.imshow(images[i], cmap='gray')
        plt.title(titles[i])

    plt.show()


if __name__ == '__main__':
    main()
