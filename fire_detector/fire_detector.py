import numpy as np
import cv2
cap = cv2.VideoCapture('/home/kicker/fire_detector/Burning wood in firepit.mp4')
lower_yellow = np.array([30,50,50])
upper_yellow = np.array([100,255,255])
while(cap.isOpened()):
    #Capture frame-by-frame
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
    res = cv2.bitwise_and(frame, hsv_frame, mask=mask)
    cv2.imshow('frame2', res)
    cv2.imwrite('cur_frame.jpg', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
