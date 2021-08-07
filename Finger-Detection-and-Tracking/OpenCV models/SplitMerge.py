#-*- coding: utf-8 -*-
import cv2
import matplotlib.pyplot as plt
'''
컬러 체널 분리하기[split] 빨강, 파랑, 초록
'''

def main():
    imageOne = cv2.imread("/home/kicker/Finger-Detection-and-Tracking/Sample images/house.tiff", 1) #이미지를 불러온다
    imageOne = cv2.cvtColor(imageOne, cv2.COLOR_BGR2RGB)  #cv 형식으로 변환하고

    red, green, blue = cv2.split(imageOne)  # 색 체널을 나눠준다

    images = [cv2.merge((red, green, blue)), red, green, blue] # 나뉜 체널에서 이미지를 뽑는다
    titles = ["Default RGB Image", "Only Red", "Only Blue", "Only Green"]
    cmaps = ["gray", "Reds", "Greens", "Blues"]

    for i in range(4):
        plt.subplot(2, 2, i + 1)

        plt.imshow(images[i], cmap=cmaps[i])
        plt.title(titles[i])
        plt.xticks([])
        plt.yticks([])

    plt.show()


if __name__ == "__main__":
    main()
