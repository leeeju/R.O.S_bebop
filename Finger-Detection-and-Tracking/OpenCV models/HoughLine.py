#-*- coding: utf-8 -*-
import cv2
import numpy as np

'''
[HoughLines] 사진 & 영상에서 직선 검출하기
영상에서 직전인 물체 또는 사진을 비춰 보세요  ex) 책, 휴대폰 등 ,,, 팔을 곧게 펴도 가능

먼저 Hough Transfom은 이미지에서 수학적으로 표현 가능한 도형을 검색하는 기술입니다.
그 도형 중 선형에 대해 검색해 볼텐데요. 선에 대한 방정식은 우리가 흔히 알고 있는 기울기(m)와 y절편(c)로 
표현되는 𝑦=m𝑥+c도 있지만 삼각함수에 의한 매개변수 방정식으로써는 r = 𝑥cos𝜃 + 𝑦sin𝜃 로도 표현됩니다.
'''

def main():
    capture = cv2.VideoCapture(0)  #켐 화면으로 보겠다

    while True:
        ret, frame = capture.read()

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        edges_detec = cv2.Canny(gray_frame, 50, 250, apertureSize=5, L2gradient=True)
                        # 엣지 검출                   #조리개 크기       # 경사도
        hough_lines = cv2.HoughLines(edges_detec, 1, np.pi / 180, 200)

        if hough_lines is not None:
            for rho, theta in hough_lines[0]:
                x0 = rho * np.cos(theta)
                y0 = rho * np.sin(theta)

                ptsX = (int(x0 + 1000 * (-np.sin(theta))), int(y0 + 1000 * (np.cos(theta))))
                ptsY = (int(x0 - 1000 * (-np.sin(theta))), int(y0 - 1000 * (np.cos(theta))))
                cv2.line(frame, ptsX, ptsY, (0, 255, 0), 2)

        cv2.imshow("Capture Frame", frame)

        if cv2.waitKey(1) == 27:
            break

    cv2.destroyAllWindows()
    capture.release()


if __name__ == '__main__':
    main()
