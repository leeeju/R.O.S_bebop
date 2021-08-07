#-*- coding: utf-8 -*-
import cv2

'''
Sobel(소벨) 필터,  Scharr(샤를) 필터

출력 이미지 정밀도는 반환되는 결과 이미지의 정밀도를 설정합니다.

X 방향 미분 차수는 이미지에서 X 방향으로 미분할 차수를 설정합니다.

Y 방향 미분 차수는 이미지에서 Y 방향으로 미분할 차수를 설정합니다.

커널 크기는 소벨 마스크의 크기를 설정합니다. 1, 3, 5, 7 등의 홀수 값을 사용하며, 최대 31까지 설정할 수 있습니다.

비율과 오프셋은 출력 이미지를 반환하기 전에 적용되며, 주로 시각적으로 확인하기 위해 사용합니다.

픽셀 외삽법은 이미지 가장자리 부분의 처리 방식을 설정합니다.

기본 개념은 어렵다. 그냥 이렇게 나오는구나~ 하면 된다.
'''

def main():
    image = cv2.imread("/home/kicker/Finger-Detection-and-Tracking/Sample images/5.1.11.tiff", 0)
    cv2.imshow("Orignal Image", image)

    # 라플라시안 하이패스 필터
    lap_filter = cv2.Laplacian(image, ddepth=-1, ksize=7, scale=1, borderType=cv2.BORDER_DEFAULT)
    cv2.imshow("Laplacian Filter", lap_filter)

    # Sobel 하이패스 필터
    sobelx_filter = cv2.Sobel(image, ddepth=-1, dx=2, dy=0, ksize=7, scale=1, borderType=cv2.BORDER_DEFAULT)
    cv2.imshow("Sobel X Filter", sobelx_filter)

    sobely_filter = cv2.Sobel(image, ddepth=-1, dx=0, dy=2, ksize=7, scale=1, borderType=cv2.BORDER_DEFAULT)
    cv2.imshow("Sobel Y Filter", sobely_filter)

    sobel_filter = sobelx_filter + sobely_filter
    cv2.imshow("Sobel Filter", sobel_filter)

    # Scharr 하이패스 필터
    scharrx_filter = cv2.Scharr(image, ddepth=-1, dx=1, dy=0, scale=1, borderType=cv2.BORDER_DEFAULT)
    cv2.imshow("Scharr X Filter", scharrx_filter)

    scharry_filter = cv2.Scharr(image, ddepth=-1, dx=0, dy=1, scale=1, borderType=cv2.BORDER_DEFAULT)
    cv2.imshow("Scharr Y Filter", scharry_filter)

    scharr_filter = scharrx_filter + scharry_filter
    cv2.imshow("Scharr Filter", scharr_filter)

    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
