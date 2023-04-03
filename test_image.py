import cv2 as cv
import cv2
import pymurapi as mur
import numpy as np

auv = mur.mur_init()


def binary_image():
    image = cv.imread('C:/Users/Admin/Desktop/find_object_2_02.04.2023.png', cv2.IMREAD_COLOR)

    gray = cv.cvtColor(image, cv2.COLOR_BGR2GRAY)

    gray_blurred = cv.medianBlur(gray, 3)

    rows = gray_blurred

    detected_circles = cv2.HoughCircles(gray_blurred,
                                        cv2.HOUGH_GRADIENT, 1, 20, param1=50,
                                        param2=30, minRadius=1, maxRadius=40)

    if detected_circles is not None:

        # Convert the circle parameters a, b and r to integers.
        detected_circles = np.uint16(np.around(detected_circles))

        for pt in detected_circles[0, :]:
            center = (pt[0], pt[1])

            # Draw the circumference of the circle.
            cv.circle(image, center, 1, (0, 255, 0), 2)

            # Draw a small circle (of radius 1) to show the center.
            radius = pt[2]
            cv.circle(image, center, radius, (0, 0, 255), 3)
            cv2.imshow("Detected Circle", image)
            cv2.waitKey(0)

    # Обнаружение с помощью цвета и обводка объекта

    """imageHSV = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    low_hsv_blue = (0, 50, 50)
    max_hsv_blue = (30, 255, 255)

    blue_hsv_mask = cv.inRange(imageHSV, low_hsv_blue, max_hsv_blue)

    cnt, _ = cv.findContours(blue_hsv_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    cv.drawContours(image, cnt, -1, (0, 0, 0), 2)"""

    # Обнаружение центра круга

    """moments = cv.moments(cnt[0])

    x = moments["m10"] / moments["m00"]
    y = moments["m01"] / moments["m00"]

    cv.circle(image, (int(x), int(y)), 4, (255, 0, 0), 3) """

    """cv.imshow('cnt', image)

    cv.waitKey(5)"""


"""if cnt:
        for c in cnt:
            area = cv.contourArea(c)
            if abs(area) < 300:
                continue
            hull = cv.convexHull(c)
            approx = cv.approxPolyDP(hull, cv.arcLength(c, True), True)
            if len(approx) == 4:
                cv.drawContours(image, [c], 0, (9, 0, 255), 3)
                ((x, y), (w, h), angle) = cv.minAreaRect(approx)
                aspect_ratio = w/float(h)
                if 0.9 <= aspect_ratio <= 1.1:
                    cv.drawContours(image, [c], 0, (255, 0, 255), 3)
                else:
                    cv.drawContours(image, [c], 0, (0, 255, 255), 3)
            if len(approx) > 5:
                cv.drawContours(image, [c], 0, (0, 255, 0), 3)
    """

while True:
    binary_image()
