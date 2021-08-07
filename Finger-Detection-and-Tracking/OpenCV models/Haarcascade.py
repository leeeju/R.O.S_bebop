#-*- coding: utf-8 -*-
import cv2

'''
얼굴 검출
얼굴, 눈,입술의 대한 데이터를 바탕으로 화면에서 추적한다
'''

def main():
    capture = cv2.VideoCapture(0)
    eye_path = "/home/kicker/Finger-Detection-and-Tracking/Harcascade classifiers/haarcascade_eye.xml"
    face_path = "/home/kicker/Finger-Detection-and-Tracking/Harcascade classifiers/haarcascade_frontalface_default.xml"

    eye_cascade = cv2.CascadeClassifier(eye_path)
    face_cascade = cv2.CascadeClassifier(face_path)

    while (True):
        _, frame = capture.read()

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        eyes = eye_cascade.detectMultiScale(gray_frame, scaleFactor=1.05, minNeighbors=5, minSize=(10,10))
        faces = face_cascade.detectMultiScale(gray_frame, scaleFactor=1.05, minNeighbors=5, minSize=(40, 40))

        print("Number of eyes : " + str(len(eyes)))    #눈
        print("Number of faces : " + str(len(faces)))  #얼굴

        for (x, y, w, h) in eyes: #사각형의 틀을 만들어줌
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        cv2.imshow("Live Capture", frame) # 화면 띄우기

        if cv2.waitKey(1) == 27:
            break

    cv2.destroyAllWindows()
    capture.release()


if __name__ == '__main__':
    main()
