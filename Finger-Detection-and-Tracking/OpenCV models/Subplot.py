import cv2
import matplotlib.pyplot as plt


def main():
    imageOne = cv2.imread("/home/kicker/Finger-Detection-and-Tracking/Sample images/4.1.01.tiff", 1)
    imageTwo = cv2.imread("/home/kicker/Finger-Detection-and-Tracking/Sample images/4.1.02.tiff", 1)

    imageNames = [imageOne, imageTwo]
    imageTitles = ["First Image", "Second Image"]

    for i in range(2):
        plt.subplot(1, 2, i + 1)
        plt.imshow(imageNames[i])
        plt.title(imageTitles[i])
        plt.xticks([])
        plt.yticks([])

    plt.show()


if __name__ == '__main__':
    main()
