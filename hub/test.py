import math

import hub


class Constants:
    POWER_PER_DEGREE_PER_SECOND = 1 / 9.3  # factor to convert from desired deg/s to power that needs to be applied
    MM_PER_DEGREE = -0.025  # how much does the rope length change per degree rotation of the motor
    POWER_MAX_PERCENTAGE = 1  # use only XX% of available motors power
    MAX_DEG_PER_S = 100 / POWER_PER_DEGREE_PER_SECOND * POWER_MAX_PERCENTAGE
    POINT_REACHED_ACCURACY_MM = 0.8  # how close (in mm) do we need to be at a point to consider it reached


class Config:
    def get_startpos_relative_to_canvas(self):
        """
        :return: The starting position of the robot on the canvas
        """
        return [0, 0]

    def get_canvas_dim(self):
        """
        :return: The canvas' dimension (width, height) in mm
        """
        return [300, 200]

    def get_anchor_distance(self):
        """
        :return: The distance between the two rope anchors
        """
        return 790

    def get_canvas_offset(self):
        """
        :return: The translation of the canvas coordinate system relative to the left anchor
        """
        return [290, 640]


class Geom:
    def __init__(self, d, offset, canvas_dim, degree_per_mm):
        """
        Calculates motor positions for coordinates
        :param d: distance d between two anchors
        :param offset: offset [x, y] of the canvas relative to the left anchor
        :param canvas_dim: dimensions of the canvas
        :param degree_per_mm: how many degrees do the motors need to turn for one mm
        """
        self.d = d
        self.offset = offset
        self.canvas_dim = canvas_dim
        self.degree_per_mm = degree_per_mm

    def get_degree(self, point):
        x = point[0]
        y = point[1]
        l = math.sqrt(
            (x * self.canvas_dim[0] + self.offset[0]) ** 2 + (
                    y * self.canvas_dim[1] + self.offset[1]) ** 2) * self.degree_per_mm
        r = math.sqrt((self.d - self.offset[0] - x * self.canvas_dim[0]) ** 2 + (
                y * self.canvas_dim[1] + self.offset[1]) ** 2) * self.degree_per_mm
        return [l, r]


class MotorController:
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
        self.port_left.pwm(round(left * Constants.POWER_PER_DEGREE_PER_SECOND))
        self.port_right.pwm(round(right * Constants.POWER_PER_DEGREE_PER_SECOND))


class PathController:
    min_steps_per_mm = 1

    def __init__(self, canvas_dim):
        self.canvas_dim = canvas_dim
        self.interpolated = []
        self.idx = 0

    def load_path(self, path):
        self.interpolated = [path[0]]
        self.idx = 0

        for p1 in path[1:]:
            p0 = self.interpolated[-1]
            dx = (p1[0] - p0[0])
            dy = (p1[1] - p0[1])
            d = math.sqrt((dx * self.canvas_dim[0]) ** 2 + (dy * self.canvas_dim[1]) ** 2)
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
    point_reached_error_threshold = abs(Constants.POINT_REACHED_ACCURACY_MM / Constants.MM_PER_DEGREE)

    def __init__(self):
        self.config = Config()
        self.mc = MotorController(hub.port.B, hub.port.F)
        self.geom = Geom(self.config.get_anchor_distance(), self.config.get_canvas_offset(),
                         self.config.get_canvas_dim(), (1 / Constants.MM_PER_DEGREE))
        self.pc = PathController(self.config.get_canvas_dim())

    def draw(self, path):
        self.pc.load_path(path)
        self.mc.preset()
        p0 = self.geom.get_degree(self.config.get_startpos_relative_to_canvas())

        run = True

        try:
            while run:
                point = self.pc.current_point()
                [left_desired_deg, right_desired_deg] = self.geom.get_degree(point)
                [left_pos, right_pos] = self.mc.get_pos()

                left_error_deg = left_desired_deg - (left_pos + p0[0])
                right_error_deg = right_desired_deg - (right_pos + p0[1])

                error = math.sqrt(left_error_deg ** 2 + right_error_deg ** 2)

                if error < self.point_reached_error_threshold:
                    # consider point reached
                    if self.pc.has_next():
                        self.pc.next()
                        continue
                    else:
                        run = False
                        continue

                if abs(left_error_deg) > abs(right_error_deg):
                    left_deg_per_s = math.copysign(Constants.MAX_DEG_PER_S, left_error_deg)
                    right_deg_per_s = right_error_deg / abs(left_error_deg) * Constants.MAX_DEG_PER_S
                else:
                    right_deg_per_s = math.copysign(Constants.MAX_DEG_PER_S, right_error_deg)
                    left_deg_per_s = left_error_deg / abs(right_error_deg) * Constants.MAX_DEG_PER_S

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
