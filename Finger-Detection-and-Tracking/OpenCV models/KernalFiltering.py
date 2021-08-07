#-*- coding: utf-8 -*-
import cv2
import numpy as np

'''
흐리기 효과 (3종)
Box Blur의 경우 인자값을 가지고 있습니다, 특정 범위내에서 반경을 바탕으로 흐릿한 값의 정도를 지정할 수 있다.

Gaussian Blur 이미지 위에 피지와 같은 반투명 재료를 놓는 것과 같으며 모든 것을 부드럽게 합니다"라고 말한다,
로우 패스 필터 유형인 가우시안 흐림 효과는 일정 범위 이상을 벗어나는 극단적인 이상값을 잘라내 이미지의 균일하지 
않은 픽셀 값을 고르게 조정합니다.

'''

def main():
    image = cv2.imread("/home/kicker/Finger-Detection-and-Tracking/Sample images/7.1.01.tiff", 1)
    '''
    # Kernal or Convolution matrix for Identity Filter

    kernal = np.array(([0, 0, 0],
                       [0, 1, 0],
                       [0, 0, 0]), np.float32)

    # Kernal or Convolution matrix for Edge Detection

    kernal = np.array(([-1, -1, -1],
                       [-1, 8, -1],
                       [-1, -1, -1]), np.float32)

    '''
    # Kernal or Convolution matrix for Box BLue Filter

    kernal = np.ones((5, 5), np.uint8) / 25
    output = cv2.filter2D(image, -1, kernal)

    # Low pass filters implementation
    box_blur = cv2.boxFilter(image, -1, (31, 31))
    simple_blur = cv2.blur(image, (21, 21))
    gaussian_blur = cv2.GaussianBlur(image, (51, 51), 0)

    cv2.imshow("Orignal Image", image)               #원본 이미지 
    cv2.imshow("Filtered Image", output)

    cv2.imshow("Box Blur", box_blur)                 #
    cv2.imshow("Simple Blur", simple_blur)           # 일반적인 블라인드 처리
    cv2.imshow("Gaussian Blur", gaussian_blur)       # 

    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
