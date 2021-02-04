import math

import hub


class Geom:
    """
    Calculates motor positions for coordinates
    Needs
        distance d between two anchors
        an offset of x and y where (0,0) should be relative to the left anchor
        the width and height in mm of the canvas
        how many degrees to we need per mm of rope
    """

    def __init__(self, d, offset_x, offset_y, width, height, degree_per_mm):
        self.d = d
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.width = width
        self.height = height
        self.degree_per_mm = degree_per_mm

    def get_degree(self, point):
        x = point[0]
        y = point[1]
        l = math.sqrt(
            (x * self.width + self.offset_x) ** 2 + (y * self.height + self.offset_y) ** 2) * self.degree_per_mm
        r = math.sqrt((self.d - self.offset_x - x * self.width) ** 2 + (
                y * self.height + self.offset_y) ** 2) * self.degree_per_mm
        return [l, r]


class MotorController:
    POWER_PER_DEGREE_PER_SECOND = 1 / 9.3

    def __init__(self, port_left, port_right):
        self.port_left = port_left
        self.port_right = port_right
        self.start_pos_left = 0
        self.start_pos_right = 0
        self.port_left.motor.mode([(1, 0), (2, 2), (3, 1), (0, 0)])
        self.port_right.motor.mode([(1, 0), (2, 2), (3, 1), (0, 0)])

    def preset(self):
        [self.start_pos_left, self.start_pos_right] = self.get_pos()

    def get_pos(self):
        return [self.port_left.motor.get()[1] - self.start_pos_left,
                self.port_right.motor.get()[1] - self.start_pos_right]

    def set_degree_per_second(self, left, right):
        self.port_left.pwm(round(left * self.POWER_PER_DEGREE_PER_SECOND))
        self.port_right.pwm(round(right * self.POWER_PER_DEGREE_PER_SECOND))


class PathController:
    min_steps_per_mm = 1

    def __init__(self, width_mm, height_mm):
        self.width_mm = width_mm
        self.height_mm = height_mm
        self.interpolated = []
        self.idx = 0

    def load_path(self, path):
        self.interpolated = [path[0]]
        self.idx = 0

        for p1 in path[1:]:
            p0 = self.interpolated[-1]
            dx = (p1[0] - p0[0])
            dy = (p1[1] - p0[1])
            d = math.sqrt((dx * self.width_mm) ** 2 + (dy * self.height_mm) ** 2)
            needed_points_for_distance = math.ceil(d * self.min_steps_per_mm)
            for j in range(1, needed_points_for_distance + 1):
                self.interpolated.append([p0[0] + dx * j / needed_points_for_distance,
                                          p0[1] + dy * j / needed_points_for_distance])

    def current_point(self):
        return self.interpolated[self.idx]

    def has_next(self):
        return self.idx < len(self.interpolated) - 1

    def next(self):
        self.idx += 1


class Plotter:
    mm_per_degree = -0.025
    max_deg_per_s = 930 * 0.9  # use only 90% of motors maximum speed
    width_mm = 300
    height_mm = 200

    def __init__(self):
        self.mc = MotorController(hub.port.B, hub.port.F)
        self.geom = Geom(790, 290, 640, self.width_mm, self.height_mm, (1 / self.mm_per_degree))
        self.pc = PathController(self.width_mm, self.height_mm)

    def draw(self, path):
        self.pc.load_path(path)
        self.mc.preset()
        # assume we start at (0,0) on our canvas. Canvas (0,0) is relative to left anchor at (offset_x, offset_y)
        p0 = self.geom.get_degree([0, 0])

        run = True

        try:
            while run:
                point = self.pc.current_point()
                [left_desired_deg, right_desired_deg] = self.geom.get_degree(point)
                [left_pos, right_pos] = self.mc.get_pos()

                left_error_deg = left_desired_deg - (left_pos + p0[0])
                right_error_deg = right_desired_deg - (right_pos + p0[1])

                error = math.sqrt(left_error_deg ** 2 + right_error_deg ** 2)

                if error < 30:
                    # consider point reached
                    if self.pc.has_next():
                        self.pc.next()
                        continue
                    else:
                        run = False
                        continue

                if abs(left_error_deg) > abs(right_error_deg):
                    left_deg_per_s = math.copysign(self.max_deg_per_s, left_error_deg)
                    right_deg_per_s = right_error_deg / abs(left_error_deg) * self.max_deg_per_s
                else:
                    right_deg_per_s = math.copysign(self.max_deg_per_s, right_error_deg)
                    left_deg_per_s = left_error_deg / abs(right_error_deg) * self.max_deg_per_s

                self.mc.set_degree_per_second(left_deg_per_s, right_deg_per_s)

        except Exception as e:
            print("Caught exception", e)

        # finished drawing or exception, stop
        self.mc.set_degree_per_second(0, 0)


path = [
    [0, 0],
    [1, 0],
    [1, 1],

    [0.5, 1],
    [0.16, 0.63],
    [0.11, 0.5],
    [0.13, 0.37],
    [0.17, 0.26],
    [0.24, 0.19],
    [0.34, 0.17],
    [0.41, 0.18],
    [0.45, 0.21],
    [0.47, 0.26],
    [0.5, 0.3],
    [0.53, 0.26],
    [0.55, 0.21],
    [0.59, 0.18],
    [0.66, 0.17],
    [0.76, 0.19],
    [0.83, 0.26],
    [0.87, 0.37],
    [0.89, 0.5],
    [0.84, 0.63],
    [0.5, 1],

    [0, 1],
    [0, 0],
]

path = [
    [0, 0],
    [1, 0],
    [0, 1],
    [1, 1],
    [0, 0],
]
# path = [[x, 1 - y] for [x, y] in path]

plotter = Plotter()
plotter.draw(path)
