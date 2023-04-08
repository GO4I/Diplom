import pymurapi as mur
import cv2 as cv
import math
import time

auv = mur.mur_init()

def clamp(v, max_v, min_v):
    if v > max_v:
        return max_v
    if v < min_v:
        return min_v
    return v

class PID(object): #класс ПИД регулятора
    _kp = 0.0
    _kd = 0.0
    _ki = 0.0
    _prev_error = 0.0
    _timestamp = 0

    def __init__(self):
        pass

    def set_p_gain(self, value):
        self._kp = value #установка пропорционального коэффициента

    def set_i_gain(self, value):
        self._ki = value #установка интегрального коэффициента

    def set_d_gain(self, value):
        self._kd = value #установка дифференциального коэффициента

    def time(self):
        return self._timestamp

    def process(self, error):
        I = 0
        timestamp = int(round(time.time() * 1000)) #текущее время

        P = error #пропорциональная составляющая
        I += (error - self._prev_error) * (timestamp - self._timestamp) #интегральная составляющая
        D = (error - self._prev_error) / (timestamp - self._timestamp) #дифференциальная составляющая

        output = self._kp * P + self._ki * I + self._kd * D #расчёт управляющего сигнала
        self._timestamp = timestamp #прошлое время
        self._prev_error = error #прошлая ошибка
        return output #возвращение выхода

# ?
def keep_depth(depth_to_set, speed):
    try:
        error = auv.get_depth() - depth_to_set
        output = keep_depth.regulator.process(error)
        output = clamp(output, 100, -100)
        auv.set_motor_power(2, speed + output)
        auv.set_motor_power(3, speed + output)
        return error
    except AttributeError:
        keep_depth.regulator = PID()
        keep_depth.regulator.set_p_gain(90)
        keep_depth.regulator.set_i_gain(30)
        keep_depth.regulator.set_d_gain(0)

def keep_yaw(yaw_to_set, speed):
    def clamp_to180(angle):
        if angle > 180.0:
            return angle - 360
        if angle < -180.0:
            return angle + 360
        return angle
    try:
        error = auv.get_yaw() - yaw_to_set
        error = clamp_to180(error)
        output = keep_yaw.regulator.process(error)
        output = clamp(output, 100, -100)
        auv.set_motor_power(0, clamp((speed - output), 100, -100))
        auv.set_motor_power(1, clamp((speed + output), 100, -100))
    except AttributeError:
        keep_yaw.regulator = PID()
        keep_yaw.regulator.set_p_gain(0.8)
        keep_yaw.regulator.set_i_gain(0)
        keep_yaw.regulator.set_d_gain(300)
# ?

def find_yellow_circle(img):
    image_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    hsv_min = (20, 0, 0)
    hsv_max = (40, 255, 255)
    image_bin = cv.inRange(image_hsv, hsv_min, hsv_max)
    cnt, _ = cv.findContours(image_bin, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    if cnt:
        for c in cnt:
            moments = cv.moments(c)

            (_, _), radius = cv.minEnclosingCircle(c)
            circle_area = radius ** 2 * math.pi

            try:
                x = int(moments["m10"] / moments["m00"])
                y = int(moments["m01"] / moments["m00"])
                return True, x, y, circle_area
            except ZeroDivisionError:
                return False, 0, 0, 0
    else:
        return False, 0, 0, 0


def stab_on_yellow_circle(image):
    found, x, y, circle_area = find_yellow_circle(image)
    speed = 10

    def clamp_to180(angle):
        if angle > 180.0:
            return angle - 360
        if angle < -180.0:
            return angle + 360
        return angl
    if found:

        image_area = 320*240
        a = image_area / circle_area

        x = x - (320 / 2)
        y = y - (240 / 2)
        roll_to_set = 0

        alfa = math.degrees(math.atan((x/160) * math.tan(59/2)))
        beta = math.degrees(math.atan((y/120) * math.tan(46/2)))
        error = auv.get_roll() - roll_to_set

        print(a)
        try:
            output_x = stab_on_yellow_circle.regulator_forward.process(alfa)
            output_y = stab_on_yellow_circle.regulator_side.process(beta)
            output_roll = stab_on_yellow_circle.regulator_roll.process(error)

            output_x = clamp(output_x, 50, -50)
            output_y = clamp(output_y, 50, -50)

            auv.set_motor_power(0, speed +output_x)
            auv.set_motor_power(1, speed -output_x)

            auv.set_motor_power(2,  -output_y)
            auv.set_motor_power(3,  -output_y)

            auv.set_motor_power(4, output_roll)
        except AttributeError:
            stab_on_yellow_circle.regulator_forward = PID()
            stab_on_yellow_circle.regulator_forward.set_p_gain(0.5)
            stab_on_yellow_circle.regulator_forward.set_i_gain(0)
            stab_on_yellow_circle.regulator_forward.set_d_gain(0)

            stab_on_yellow_circle.regulator_side = PID()
            stab_on_yellow_circle.regulator_side.set_p_gain(0.5)
            stab_on_yellow_circle.regulator_side.set_i_gain(0)
            stab_on_yellow_circle.regulator_side.set_d_gain(0)

            stab_on_yellow_circle.regulator_roll = PID()
            stab_on_yellow_circle.regulator_roll.set_p_gain(0.5)
            stab_on_yellow_circle.regulator_roll.set_i_gain(0)
            stab_on_yellow_circle.regulator_roll.set_d_gain(0)
    return False



while True:
    image = auv.get_image_front()
    stab_on_yellow_circle(image)
    #keep_depth(1.5, 0)
    time.sleep(0.05)

