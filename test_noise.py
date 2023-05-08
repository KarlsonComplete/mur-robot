import random
import cv2 as cv
import cv2
import numpy as np


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


image = cv.imread('C:/Users/Admin/Desktop/find_object_2_02.04.2023.png', cv2.IMREAD_COLOR)
cv.imshow("Source Image", image)
out1 = sp_noise(image, 0.3)
out2 = gasuss_noise(image, mean=0, var=0.01)
cv.imshow("sp_noise", out1)
cv.imshow("gauss_image", out2)
cv2.waitKey()
