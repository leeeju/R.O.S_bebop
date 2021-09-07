#-*- coding: utf-8 -*- 

import cv2, sys
import numpy as np
import smtplib
import playsound
import threading

Alarm_Status = False
Email_Status = False
Fire_Reported = 0

fName = sys.argv[1] #쉘(shell)에서 인자를 받는다.

def play_alarm_sound_function():
	while True:
		playsound.playsound('/home/kicker/catkin_ws/alarm.mp3',True)

def send_mail_function():

    recipientEmail = "Enter_Recipient_Email"
    recipientEmail = recipientEmail.lower()

    try:
        server = smtplib.SMTP('smtp.gmail.com', 587)
        server.ehlo()
        server.starttls()
        server.login("Enter_Your_Email (System Email)", 'Enter_Your_Email_Password (System Email')
        server.sendmail('Enter_Your_Email (System Email)', recipientEmail, "Warning A Fire Accident has been reported on ABC Company")
        print("sent to {}".format(recipientEmail))
        server.close()
    except Exception as e:
    	print(e)
 
while True:

    unchange = cv2.imread(fName, cv2.IMREAD_UNCHANGED) # cv2.imread()는 이미지 파일을 읽어 들이는 함수이다. 터미널에서 받은 인자를 이미지로 받는 객체 unchange를 만든다.
    frame = cv2.imread(fName)
    
    frame = cv2.resize(frame, (640, 480)) # 사진 크기 조절.
 
    blur = cv2.GaussianBlur(frame, (21, 21), 0) # 가우시안 블러링(Gaussian Blurring)이란 그림의 중심에 높은 가중치를 부여하는 블러링이다. 블러링은 이미지를 흘리게 처리하는 기법이다. 
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV) # cv2.cvtColor()는 이미지의 색상을 바꾸는 함수이다. 
 
    lower = [18, 50, 50]   # RGB로 변환하면 [64, 128, 128]이다. 대략 진한 녹색에 가까운 색이다.
    upper = [35, 255, 255] # RGB로 변환하면 [255, 25, 255]이다. 대략 진한 핑크색에 가까운 색이다.

    lower = np.array(lower, dtype="uint8")
    upper = np.array(upper, dtype="uint8")

    fire = cv2.inRange(hsv, lower, upper) #cv.inRange()는 어떤 요소가 서로 다른 두 배열의 사이에 있는지 확인하는 함수이다. 불꽃에서 대체로 볼 수 있는 생상으로 범위를 한정해준다.
 
    output = cv2.bitwise_and(frame, hsv, mask=fire) #cv.bitwise_and()는 두 이미지의 비트 and연산을 하는 함수이다. 앤드 연산을 하면 색깔이 없는 영역은 지워진다. 즉, 불꽃 영역에 한정된 색상 이외의 영역은 지운다.
    
    no_red = cv2.countNonZero(fire) #영이 아닌 매트릭스를 세는 함수
    
    if int(no_red) > 15000:
        Fire_Reported = Fire_Reported + 1

    cv2.imshow("output", output)

    cv2.imshow(fName, unchange)

    if Fire_Reported >= 1:
    
    	if Alarm_Status == False:
    		threading.Thread(target=play_alarm_sound_function).start()
    		Alarm_Status = True

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
