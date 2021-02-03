import hub
import math
import time

def clamp(value, min_value, max_value):
    return min(max(round(value), min_value), max_value)

class PID:
    """
    Simple proportional–integral–derivative (PID) controller
    """
    def __init__(self, kp, ki, kd):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self.reset()

    def process(self, setpoint, process_value, dt):
        error = setpoint - process_value
        self._integral += error * dt
        self._derivative = (error - self._error_prev) / dt
        self._error_prev = error
        return self._kp * error + self._ki * self._integral + self._kd * self._derivative

    def reset(self):
        self._derivative = 0
        self._integral = 0
        self._error_prev = 0

class Geom:
    """
    Calculates motor positions for coordinates
    Needs
        distance d between two anchors
        an offset of x and y where (0,0) should be relative to the left anchor
        the width and height in mm of the canvas
        scale to convert a desired length to motor angle in mm/degree
    """
    def __init__(self, d, offsetX, offsetY, width, height, scale):
        self.d = d
        self.offsetX = offsetX
        self.offsetY = offsetY
        self.width = width
        self.height = height
        self.scale = scale

    def get_angle(self, x, y):
        l = math.sqrt((x * self.width + self.offsetX) ** 2 + (y * self.height + self.offsetY) ** 2) * self.scale
        r = math.sqrt( (self.d - self.offsetX - x * self.width) ** 2 + (y * self.height + self.offsetY) ** 2) * self.scale
        return [l, r]


class Controller:
    

    mm_per_degree = -0.025
    deg_per_s_max = 1020

    def __init__(self):
        self.geom = Geom(120, 10, 10, 270, 400, 1)
        print(self.geom.get_angle(0.1, 0.1))

hub.port.E.motor.mode([(1,0), (2,2), (3,1), (0,0)])


lastTime = time.ticks_ms()
lastPos = hub.port.E.motor.get()[1]
nextPrint = lastTime + 1000

pid = PID(0.00098, 0.002, 0)
last_ticks_us = time.ticks_us()

last_abs_pos = hub.port.E.motor.get()[1]
pwm = 0
rolling_avg = 0
while True:

    ticks_us = time.ticks_us()
    dt = time.ticks_diff(ticks_us, last_ticks_us) * 0.000001
    last_ticks_us = ticks_us

    setpoint_dps = 500

    abs_pos = hub.port.E.motor.get()[1]
    dps = hub.port.E.motor.get()[0] * 9.3
    last_abs_pos = abs_pos

    if rolling_avg == 0:
        rolling_avg = dps
    rolling_avg = rolling_avg * 0.99 + 0.01 * dps

    feedback = pid.process(setpoint_dps, dps, dt)
    pwm += feedback
    hub.port.E.motor.pwm(clamp(pwm + feedback, -100, 100))
    # hub.port.E.motor.run_at_speed(round(setpoint_dps / 9.3))

    if(time.ticks_ms() > nextPrint):
        now = time.ticks_ms()
        nextPrint = now + 1000
        nowPos = hub.port.E.motor.get()[1]
        lasttimeFrameDps = (nowPos - lastPos) / ((now - lastTime) / 1000)
        speed = hub.port.E.motor.get()[0]
        print("dps:", lasttimeFrameDps, "rolling avg", rolling_avg)
        lastTime = now
        lastPos = nowPos