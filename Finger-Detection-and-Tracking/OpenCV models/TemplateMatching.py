#-*- coding: utf-8 -*-
import cv2

'''
템플릿 매칭 = 원본 이미지에서 템플릿 이미지와 일치하는 영역을 찾는 알고리즘입니다
예시로 있는 축구 사진에서 템플릿 이미지인 축구공 사진을 일치하는 곳으로 보고 표시됨
'''

def main():
    frame = cv2.imread("/home/kicker/Finger-Detection-and-Tracking/Sample images/players.jpg", 0)  #축구하는 사진
    template = cv2.imread("/home/kicker/Finger-Detection-and-Tracking/Sample images/template.jpg", 0) # 축구공 사진

    result = cv2.matchTemplate(frame, template, cv2.TM_CCOEFF_NORMED)

    min_value, max_value, min_loc, max_loc = cv2.minMaxLoc(result)

    cv2.circle(result, max_loc, 20, 255, 1) #마커 표시

    cv2.imshow("Frame Image", frame)
    cv2.imshow("Result Image", result)

    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
