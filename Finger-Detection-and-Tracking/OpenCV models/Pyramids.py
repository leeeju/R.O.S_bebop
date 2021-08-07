import cv2
'''
라플라시안 연산 이미지 
'''

def main():
    image = cv2.imread("/home/kicker/Finger-Detection-and-Tracking/Sample images/4.2.03.tiff", 1)

    first_layer_down = cv2.pyrDown(image)
    first_layer_up = cv2.pyrUp(first_layer_down)

    laplasian = cv2.subtract(image, first_layer_up)

    cv2.imshow("Orignal Image", image)
    cv2.imshow("Laplasian Image", laplasian)

    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
