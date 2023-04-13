import pymurapi as mur
import cv2 as cv
import numpy as numpy
import matplotlib.pyplot as plt
import math
import time
import enum


auv = mur.mur_init()

class Data(enum.IntEnum):
    hight_image = 320
    weight_image = 240
    vertical_angle = 46
    horizontal_angle = 59
    hue_min = 20
    hue_max = 40


def invert(v):
    if (v >= 0):
        v * 1
    else:
        v * -1
    return v

def clamp(v, max_v, min_v):
    if v > max_v:
        return max_v
    if v < min_v:
        return min_v
    return v

def clamp_to180(angle):
    if angle > 180.0:
        return angle - 360
    if angle < -180.0:
        return angle + 360
    return angle

def flow():
    mean = 0
    std = 1
    num_samples = 1000
    samples = numpy.random.normal(mean, std, size=num_samples)

    plt.plot(samples)
    plt.show(block=True)

class PID(object):  # класс ПИД регулятора
    _kp = 0.0
    _kd = 0.0
    _ki = 0.0
    _prev_error = 0.0
    _prev_PV = 0.0
    _timestamp = 0

    def __init__(self):
        pass

    def set_p_gain(self, value):
        self._kp = value  # установка пропорционального коэффициента

    def set_i_gain(self, value):
        self._ki = value  # установка интегрального коэффициента

    def set_d_gain(self, value):
        self._kd = value  # установка дифференциального коэффициента

    def time(self):
        return self._timestamp

    def process(self, error, PV):
        I = 0
        timestamp = int(round(time.time() * 1000))  # текущее время

        P = error  # пропорциональная составляющая

        I += (error - self._prev_error) * (timestamp - self._timestamp)  # интегральная составляющая

        #D = (error - self._prev_error) / (timestamp - self._timestamp)  # дифференциальная составляюща
        D = -(PV - self._prev_PV) / (timestamp - self._timestamp)

        output = self._kp * P + self._ki * I + self._kd * D  # расчёт управляющего сигнала
        self._timestamp = timestamp  # прошлое время
        self._prev_error = error  # прошлая ошибка
        self._prev_PV = PV # прошлое значение сигнала
        return output  # возвращение выхода

def find_yellow_circle(img):
    image_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    hsv_min = (Data.hue_min, 0, 0)
    hsv_max = (Data.hue_max, 255, 255)
    image_bin = cv.inRange(image_hsv, hsv_min, hsv_max)
    cnt, _ = cv.findContours(image_bin, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    if cnt:
        for c in cnt:
            hull = cv.convexHull(c)
            approx = cv.approxPolyDP(hull, cv.arcLength(c, True) * 0.02, True)
            x, y, w, h = cv.boundingRect(approx)
            area_approx_rectangle = w / h

            moments = cv.moments(c)

            (_, _), radius = cv.minEnclosingCircle(c)
            circle_area = radius ** 2 * math.pi

            try:
                x = int(moments["m10"] / moments["m00"])
                y = int(moments["m01"] / moments["m00"])
                return True, x, y, circle_area, area_approx_rectangle
            except ZeroDivisionError:
                return False, 0, 0, 0, 1
    else:
        return False, 0, 0, 0, 1


def stab_on_yellow_circle(image):
    found, x, y, circle_area, area_approx_rectangle = find_yellow_circle(image)

    if found:

        image_area = Data.hight_image * Data.weight_image
        area_circle = image_area / circle_area

        if (area_circle <= 3):
            return True, 0

        print(area_circle)

        x = x - (Data.hight_image / 2)
        y = y - (Data.weight_image / 2)

        alfa = math.degrees(math.atan((x / (Data.hight_image / 2)) * math.tan(Data.horizontal_angle / 2)))
        beta = math.degrees(math.atan((y / (Data.weight_image / 2)) * math.tan(Data.vertical_angle / 2)))

        error_approx_rectangle = 1 - area_approx_rectangle

        try:
            output_x = stab_on_yellow_circle.regulator_x.process(alfa, x)
            output_y = stab_on_yellow_circle.regulator_y.process(beta, y)
            output_speed = stab_on_yellow_circle.regulator_speed.process(area_circle, 0)
            output_roll = stab_on_yellow_circle.regulator_roll.process(error_approx_rectangle, area_approx_rectangle)

            output_x = clamp(output_x, 50, -50)
            output_y = clamp(output_y, 50, -50)
            speed = clamp(output_speed, 50, -50)

            auv.set_motor_power(0,  speed + output_x)
            auv.set_motor_power(1,  speed - output_x)

            auv.set_motor_power(2, -output_y)
            auv.set_motor_power(3, -output_y)

            auv.set_motor_power(4, invert(output_roll))

        except AttributeError:
            stab_on_yellow_circle.regulator_x = PID()
            stab_on_yellow_circle.regulator_x.set_p_gain(0.5)
            stab_on_yellow_circle.regulator_x.set_i_gain(0)
            stab_on_yellow_circle.regulator_x.set_d_gain(1)

            stab_on_yellow_circle.regulator_y = PID()
            stab_on_yellow_circle.regulator_y.set_p_gain(0.5)
            stab_on_yellow_circle.regulator_y.set_i_gain(0)
            stab_on_yellow_circle.regulator_y.set_d_gain(1)

            stab_on_yellow_circle.regulator_speed = PID()
            stab_on_yellow_circle.regulator_speed.set_p_gain(0.5)
            stab_on_yellow_circle.regulator_speed.set_i_gain(0)
            stab_on_yellow_circle.regulator_speed.set_d_gain(1)

            stab_on_yellow_circle.regulator_roll = PID()
            stab_on_yellow_circle.regulator_roll.set_p_gain(100)
            stab_on_yellow_circle.regulator_roll.set_i_gain(0)
            stab_on_yellow_circle.regulator_roll.set_d_gain(0)
        return False, area_circle  #возвражение ошибки для построения графика
    return False, 0

y = []

tic = time.perf_counter()
while True:
    image = auv.get_image_front()
    done , error = (stab_on_yellow_circle(image))
    y.append(error)
    if done:
        break
    time.sleep(0.05)
toc = time.perf_counter()
time_execution = toc - tic

print(toc - tic)

x = numpy.arange(0.0, time_execution, time_execution/len(y))

plt.plot(x,y)
plt.show()