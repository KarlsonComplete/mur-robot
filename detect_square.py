import cv2 as cv
import cv2
import pymurapi as mur
import numpy as np
import random


def detect_square(max_slider=3):
    img = cv2.imread('C:/Users/Admin/Desktop/detects_circles/find_object_2_02.04.2023.png', cv2.IMREAD_COLOR)
    # Convert the image to gray-scale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the edges in the image using canny detector
    edges = cv2.Canny(gray, 50, 200)
    # Detect points that form a line
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, max_slider, minLineLength=10, maxLineGap=250)
    # Draw lines on the image
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 3)
    # Show result
    cv2.imshow("Result Image", img)
    cv2.waitKey(0)


def detect_square_new():
    hsv_min = (0, 0, 0)
    hsv_max = (30, 255, 255)

    img = cv2.imread('C:/Users/Admin/Desktop/detects_circles/find_object_2_02.04.2023.png')
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    thresh = cv.inRange(hsv, hsv_min, hsv_max)
    cnt, _ = cv.findContours(thresh.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    for c in cnt:
        rect = cv.minAreaRect(c)
        box = cv.boxPoints(rect)
        box = np.int0(box)
        cv.drawContours(img, [box], 0, (255, 0, 0), 2)
    cv.imshow('contours', img)
    cv.waitKey()


def detect():
    img = cv2.imread('C:/Users/Admin/Desktop/detects_circles/find_object_2_02.04.2023.png')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, thresh = cv2.threshold(gray, 127, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, 1, 2)
    print("Number of contours detected:", len(contours))

    for cnt in contours:
        x1, y1 = cnt[0][0]
        approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(cnt)
            ratio = float(w) / h
            if 0.9 <= ratio <= 1.1:
                img = cv2.drawContours(img, [cnt], -1, (0, 255, 255), 3)
                cv2.putText(img, 'Square', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            else:
                cv2.putText(img, 'Rectangle', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                img = cv2.drawContours(img, [cnt], -1, (0, 255, 0), 3)

    cv2.imshow("Shapes", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def detect2():
    img = cv2.imread("C:/Users/Admin/Desktop/detects_circles/find_object_2_02.04.2023.png")

    # convert to HSV, since red and yellow are the lowest hue colors and come before green
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # create a binary thresholded image on hue between red and yellow
    lower = (0, 240, 160)
    upper = (30, 255, 255)
    thresh = cv2.inRange(hsv, lower, upper)

    # apply morphology
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    clean = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    clean = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    # get external contours
    contours = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]

    result1 = img.copy()
    result2 = img.copy()
    for c in contours:
        cv2.drawContours(result1, [c], 0, (0, 0, 0), 2)
        # get rotated rectangle from contour
        rot_rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rot_rect)
        box = np.int0(box)
        # draw rotated rectangle on copy of img
        cv2.drawContours(result2, [box], 0, (0, 0, 0), 2)

    # save result
    cv2.imwrite("4cubes_thresh.jpg", thresh)
    cv2.imwrite("4cubes_clean.jpg", clean)
    cv2.imwrite("4cubes_result1.png", result1)
    cv2.imwrite("4cubes_result2.png", result2)

    # display result
    cv2.imshow("thresh", thresh)
    cv2.imshow("clean", clean)
    cv2.imshow("result1", result1)
    cv2.imshow("result2", result2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


while True:
    detect()
