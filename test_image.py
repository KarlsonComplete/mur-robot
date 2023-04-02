import cv2 as cv
import pymurapi as mur

auv = mur.mur_init()

def binary_image():
    image = cv.imread('C:/Users/Admin/Desktop/find_object_2_02.04.2023.png')

    imageHSV = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    low_hsv_blue = (0, 50, 50)
    max_hsv_blue = (30, 255, 255)

    blue_hsv_mask = cv.inRange(imageHSV, low_hsv_blue, max_hsv_blue)

    cv.imshow('Window', blue_hsv_mask)

    cv.waitKey(5)

while True:
    binary_image()