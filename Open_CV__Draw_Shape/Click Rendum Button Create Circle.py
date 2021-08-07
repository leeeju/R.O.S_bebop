import numpy as np
import cv2
from random import shuffle  #색을 랜덤으로 뽑는다 

b =[i for i in range(256)] #블루
g =[i for i in range(256)] #그린
r =[i for i in range(256)] #레드

'''
마우스 더블 클릭으로 CV 화면중 임의 위치에 원(점) 생성하기
더블 클릭한 곳에 원 생성
'''

def onMouse(event, x, y, flags, param): #이벤트 발생 (EVENT_LBUTTONDBLCLK) 마우스 버튼 더블 클릭
    if event == cv2.EVENT_LBUTTONDBLCLK:
       shuffle(b), shuffle(g), shuffle(r)
       cv2.circle(param, (x, y), 50, (b[0], g[0], r[0]), -1)

def mouseBrush(): #렌덤 하게 클릭된 위치게 원을 생성한다
    img = np.zeros((512, 512, 3), np.uint8)
    cv2.namedWindow('paint')
    cv2.setMouseCallback( 'paint', onMouse, param=img)
    
    while True:
        cv2.imshow('paint', img)
        k = cv2.waitKey(1) & 0xFF

        if k == 27:
           break
    cv2.destroyAllWindows()

mouseBrush()