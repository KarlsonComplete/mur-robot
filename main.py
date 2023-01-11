import pymurapi as mur
import time

auv = mur.mur_init()


def clamp_to180(angle):
    if angle > 180.0:
        return angle - 360
    if angle < 180.0:
        return angle + 360
    return angle


def clamp(v, max_v, min_v):
    if v > max_v:
        return max_v
    if v < min_v:
        return min_v
    return v


class PD(object):
    _kp = 0.0
    _kd = 0.0
    _prev_error = 0.0
    _timestamp = 0.0

    def __init__(self):
        pass

    # Метод для устанвоки пропорционального усиления
    def set_p_gain(self, value):
        self._kp = value

    # Метод для устанвоки дифиринциального усиления
    def set_d_gain(self, value):
        self._kd = value

    # Метод для расчета регулирующего воздействия
    def process(self, error):
        timestamp = int(round(time.time() * 1000))
        output = self._kp * error + self._kd / (timestamp - self._timestamp) * (error - self._prev_error)
        self._timestamp = timestamp
        self._prev_error = error
        return output


def keep_depth(depth_to_set):
    try:
        error = auv.get_depth() - depth_to_set
        output = keep_depth.regulator.process(error)
        output = clamp(output, 100 , -100)
        auv.set_motor_power(2, output)
        auv.set_motor_power(3, output)
    except AttributeError:
        keep_depth.regulator = PD()
        keep_depth.regulator.set_p_gain(50)
        keep_depth.regulator.set_d_gain(2)


while True:
    keep_depth(2)
    time.sleep(0.03)
