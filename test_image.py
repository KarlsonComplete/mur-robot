import cv2 as cv
import cv2
import pymurapi as mur
import numpy as np
import random
import math

auv = mur.mur_init()

image = cv.imread('C:/Users/Admin/Desktop/detects_circles/find_object_2_02.04.2023.png')


def gasuss_noise(image_gauss, mean=0, var=0.001):
    """
        Добавить гауссовский шум
                 Среднее значение: среднее
                 Вар: Разнообразие
    """

    image_gauss = np.array(image_gauss / 255, dtype=float)
    noise = np.random.normal(mean, var ** 0.5, image_gauss.shape)
    out = image_gauss + noise
    if out.min() < 0:
        low_clip = -1.
    else:
        low_clip = 0.
    out = np.clip(out, low_clip, 1.0)
    out = np.uint8(out * 255)

    return out


def sp_noise(input_image, prob):
    output = np.zeros(input_image.shape, np.uint8)
    thres = 1 - prob

    for i in range(input_image.shape[0]):
        for j in range(input_image.shape[1]):
            rdn = random.random()
            if rdn < prob:
                output[i][j] = 0
            elif rdn > thres:
                output[i][j] = 255
            else:
                output[i][j] = input_image[i][j]

    return output


def binary_image():
    image = cv.imread('C:/Users/Admin/Desktop/detects_circles/photo_2023-02-20_23-17-21.jpg', cv2.IMREAD_COLOR)

    image_with_median_filter = cv2.medianBlur(image, 5)

    # Изменяем размер изображения
    final_wide = 300
    r = float(final_wide) / image_with_median_filter.shape[1]
    dim = (final_wide, int(image_with_median_filter.shape[0] * r))

    # уменьшаем изображение до подготовленных размеров
    original_resized = cv2.resize(image_with_median_filter, dim, interpolation=cv2.INTER_AREA)

    cv2.imshow("Original Resize image", original_resized)

    gray = cv.cvtColor(image_with_median_filter, cv2.COLOR_BGR2GRAY)

    ret, binary = cv.threshold(gray, 0, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)

    print("threshold：%s" % ret)

    # уменьшаем изображение до подготовленных размеров
    original_resized = cv2.resize(binary, dim, interpolation=cv2.INTER_AREA)

    cv.imshow("OTSU", original_resized)

    gray_blurred = cv2.blur(gray, (3, 3))

    detected_circles = cv2.HoughCircles(binary,
                                        cv2.HOUGH_GRADIENT, 1, 20, param1=50,
                                        param2=30, minRadius=1, maxRadius=50)

    if detected_circles is not None:

        # Конвертируем параметры окружности a,b,r в целые числа.
        detected_circles = np.uint16(np.around(detected_circles))

        for pt in detected_circles[0, :]:
            center = (pt[0], pt[1])

            # Рисуем окружность круга.
            cv.circle(binary, center, 1, (0, 255, 0), 2)

            # Рисуем маленький круг(радиусом 1), чтобы отобразить центр.
            radius = pt[2]
            cv.circle(binary, center, radius, (0, 0, 255), 3)

            final_wide = 300
            r = float(final_wide) / binary.shape[1]
            dim = (final_wide, int(binary.shape[0] * r))

            # уменьшаем изображение до подготовленных размеров
            resized = cv2.resize(binary, dim, interpolation=cv2.INTER_AREA)

            cv2.imshow("Resize image", resized)
            # cv2.imshow("Detected Circle", image)

            cv2.waitKey(0)


def gradient_image():
    image = cv.imread('C:/Users/Admin/Desktop/detects_circles/5JNfqxngZ_k.jpg', cv2.IMREAD_COLOR)

    image_with_gasuss_noise = sp_noise(image, 0.07)

    image_with_median_filter = cv2.medianBlur(image_with_gasuss_noise, 5)

    # Изменяем размер изображения параметр final wide показывает ширину изображения, после чего фотография
    # формируется по соотношению сторон.
    final_wide = 500
    r = float(final_wide) / image_with_median_filter.shape[1]
    dim = (final_wide, int(image_with_median_filter.shape[0] * r))

    # уменьшаем изображение до подготовленных размеров
    original_resized = cv2.resize(image_with_median_filter, dim, interpolation=cv2.INTER_AREA)

    cv2.imshow("Original Resize image", original_resized)

    gray = cv.cvtColor(image_with_median_filter, cv2.COLOR_BGR2GRAY)

    gray_blurred = cv2.blur(gray, (3, 3))

    detected_circles = cv2.HoughCircles(gray_blurred,
                                        cv2.HOUGH_GRADIENT, 1, 20, param1=50,
                                        param2=10, minRadius=1, maxRadius=100)

    if detected_circles is not None:

        # Конвертируем параметры окружности a,b,r в целые числа.
        detected_circles = np.uint16(np.around(detected_circles))

        for pt in detected_circles[0, :]:
            center = (pt[0], pt[1])

            # Рисуем окружность круга.
            cv.circle(image_with_median_filter, center, 1, (0, 255, 0), 2)

            # Рисуем маленький круг(радиусом 1), чтобы отобразить центр.
            radius = pt[2]
            cv.circle(image_with_median_filter, center, radius, (0, 0, 255), 3)

            # Изменяем размер изображения параметр final wide показывает ширину изображения, после чего фотография
            # формируется по соотношению сторон.
            final_wide = 500
            r = float(final_wide) / image_with_median_filter.shape[1]
            dim = (final_wide, int(image_with_median_filter.shape[0] * r))

            # уменьшаем изображение до подготовленных размеров
            resized = cv2.resize(image_with_median_filter, dim, interpolation=cv2.INTER_AREA)

            cv2.imshow("Resize image", resized)
            # cv2.imshow("Detected Circle", image)

            cv2.waitKey(0)

    # Обнаружение с помощью цвета и обводка объекта


def find_yellow_circle(img):
    imageHSV = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    low_hsv_blue = (0, 0, 0)
    max_hsv_blue = (60, 255, 255)

    blue_hsv_mask = cv.inRange(imageHSV, low_hsv_blue, max_hsv_blue)

    cnt, _ = cv.findContours(blue_hsv_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    cv.drawContours(image, cnt, -1, (0, 0, 0), 2)

    cv.imshow('cnt', image)

    cv.waitKey(5)

    # Обнаружение центра круга

    """moments = cv.moments(cnt[0])

    x = moments["m10"] / moments["m00"]
    y = moments["m01"] / moments["m00"]

    cv.circle(image, (int(x), int(y)), 4, (255, 0, 0), 3)

    cv.imshow('cnt', image)

    if cnt:
        for c in cnt:
            area = cv.contourArea(c)
            if abs(area) < 300:
                continue
            hull = cv.convexHull(c)
            approx = cv.approxPolyDP(hull, cv.arcLength(c, True), True)
            if len(approx) == 4:
                cv.drawContours(image, [c], 0, (9, 0, 255), 3)
                ((x, y), (w, h), angle) = cv.minAreaRect(approx)
                aspect_ratio = w / float(h)
                if 0.9 <= aspect_ratio <= 1.1:
                    cv.drawContours(image, [c], 0, (255, 0, 255), 3)
                else:
                    cv.drawContours(image, [c], 0, (0, 255, 255), 3)
            if len(approx) > 5:
                cv.drawContours(image, [c], 0, (0, 255, 0), 3)

    cv.waitKey(5)
"""

while True:
    find_yellow_circle(image)
