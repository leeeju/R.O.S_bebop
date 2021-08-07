#-*- coding: utf-8 -*-
import cv2

'''
윤곽선(엣지) 추출하기  == Canny
'''

def main():
    image = cv2.imread("/home/kicker/Finger-Detection-and-Tracking/Sample images/4.2.07.tiff", 1) 
    # 사진 저장 경로 설정
    cv2.imshow("Orignal Image", image)
    #원본 이미지 띄우기
    output = cv2.Canny(image, 100, 151, apertureSize=3, L2gradient=True) 
    #                 (img ,min, max) 조졸하면 출력되는 엣지 비율이 달라짐
    cv2.imshow("Edge Detected Image", output)

    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
