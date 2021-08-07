#-*- coding: utf-8 -*-
import cv2

'''
이미지 융합 및 복원
'''

def main():
    image = cv2.imread("/home/kicker/Finger-Detection-and-Tracking/Sample images/Damaged Image.tiff", 1) #원본 이미지
    mask_image = cv2.imread("/home/kicker/Finger-Detection-and-Tracking/Sample images/Mask.tiff", 0)     # 합칠 이미지

    telea_image = cv2.inpaint(image, mask_image, 5, cv2.INPAINT_TELEA)  #이미지 변환
    ns_image = cv2.inpaint(image, mask_image, 5, cv2.INPAINT_NS)        #이미지 변환

    cv2.imshow("Orignal Image", image)               # 융합된 이미지
    cv2.imshow("Mask Image", mask_image)             #융합용 이미지

    cv2.imshow("TELEA Restored Image", telea_image)  #원본
    cv2.imshow("NS Restored Image", ns_image)        # 재복원 이미지

    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
