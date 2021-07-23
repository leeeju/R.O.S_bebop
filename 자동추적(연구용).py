'''
Main
Drives the program.
import camera as cm
import StepperControl
motor_test = StepperControl.StepperMotor(12, 16, 20, 21)
while( True ):
    motor_test.forward(int((360 / 5.625) * 64))
    motor_test.reverseDirection()
'''
# import the necessary packages
import numpy as np
import cv2


# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

cv2.startWindowThread()

# open webcam video stream
cap = cv2.VideoCapture(0)

# the output will be written to output.avi
out = cv2.VideoWriter(
    'output.avi',
    cv2.VideoWriter_fourcc(*'MJPG'),
    15.,
    (640,480))

x_offset = 0

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    width = 640
    # resizing for faster detection
    frame = cv2.resize(frame, (width, 480))
    # using a greyscale picture, also for faster detection
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    # detect people in the image
    # returns the bounding boxes for the detected objects
    boxes, weights = hog.detectMultiScale(frame, winStride=(8,8) )

    boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])

    for (xA, yA, xB, yB) in boxes:
        # display the detected boxes in the colour picture
        cv2.rectangle(frame, (xA, yA), (xB, yB), (0, 255, 0), 2)
       
    new_x_offset = 0
    if(len(boxes) > 0):
        for box in boxes:
            new_x_offset += (box[0] + box[2]) / 2
        new_x_offset = new_x_offset / len(boxes)
        if(new_x_offset > width / 2):
            x_offset = -1
        else:
            x_offset = 1
   
    # Write the output video 
    out.write(frame.astype('uint8'))
    # Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
# and release the output
out.release()
# finally, close the window
cv2.destroyAllWindows()
cv2.waitKey(1)
