#-*- coding: utf-8 -*-
import random
import cv2
import numpy as np
'''
실행 할때 마다 색이 랜덤으로 변환됨
도형의 면적은 구한다 [contourArea] , 도형의 둘레를 구한다 [arcLength]
'''

def main():
    kernal = np.ones((5, 5), np.uint8)
    image = cv2.imread("/home/kicker/Finger-Detection-and-Tracking/Sample images/fuzzy.png", 1)

    # 이미지를 그레이 스케일로 변환
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 배경을 위해 이미지를 다른 개체로 임계값 지정
    _, thresh = cv2.threshold(gray_image, 50, 255, cv2.THRESH_BINARY_INV)

    # 객체에서 노이즈를 제거하기 위해 이미지 확장
    dilated_image = cv2.dilate(thresh, kernal, iterations=2)

    # fuzzy.png 이미지에서 모든 등고선 찾기
    contours, _ = cv2.findContours(dilated_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # 등고선 객체를 그릴 새 이미지
    sample = np.zeros((image.shape[0], image.shape[1], 3), np.uint8)

    for cnt in contours:

        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))

        # 'contourArea' 방법을 사용하여 등고선 면적 가져오기
        area_cnt = cv2.contourArea(cnt)

        # 'arcLength'를 사용하여 등고선 둘레를 가져옵니다.
        perimeter_cnt = cv2.arcLength(cnt, True)

        if int(area_cnt) > 1000:
            cv2.drawContours(sample, [cnt], -1, color, -1)

        print("Area : {}, Perimeter : {}".format(area_cnt, perimeter_cnt))
       #      면적 :       둘레:
    cv2.imshow("Contoured Image", sample)

    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
