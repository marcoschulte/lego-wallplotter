import math
import sys

import hub


class Constants:
    POWER_PER_DEGREE_PER_SECOND = 1 / 9.3  # factor to convert from desired deg/s to power that needs to be applied
    MM_PER_DEGREE = -3825 / 132757  # how much does the rope length change per degree rotation of the motor
    POWER_MAX_PERCENTAGE = 0.9  # use only XX% of available motors power
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
        return [60, 60]

    def get_anchor_distance(self):
        """
        :return: The distance between the two rope anchors in mm
        """
        return 1065

    def get_canvas_offset(self):
        """
        :return: The translation of the canvas coordinate system relative to the left anchor
        """
        return [140, 670]


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


class PathReader:
    def try_next_point(self):
        pass

    def current_point(self):
        pass


class PathListReader(PathReader):
    def __init__(self, points):
        self.points = points
        self.idx = 0

    def try_next_point(self):
        self.idx += 1
        return self.idx < len(self.points) - 1

    def current_point(self):
        return self.points[self.idx]


class PathFileReader(PathReader):
    min_steps_per_mm = 1

    def __init__(self, canvas_dim):
        self.canvas_dim = canvas_dim
        self.reset()

    def reset(self):
        self._point_queue = []
        self._last_point_in_path = None
        self._has_next_path = False
        self._eof = False

    def load_path(self, file):
        self.reset()
        self._fh = open(file, 'r')
        self._has_next_path = True
        self.__read_next_point()

    def __read_next_point(self):
        line = False
        last_tell = self._fh.tell()

        while line is False or len(line) == 0:
            line = self._fh.readline().strip()
            new_tell = self._fh.tell()
            if last_tell == new_tell:
                # reached eof
                self._eof = True
                return
            else:
                last_tell = new_tell

            if len(line) == 0:
                # start of new path or eof if linebreak is at end of file
                self._last_point_in_path = None
                self._has_next_path = True

        p = [float(x) for x in line.split(",")]

        to_add = []
        if self._last_point_in_path is None:
            to_add.append(p)
        else:
            p0 = self._last_point_in_path
            dx = (p[0] - p0[0])
            dy = (p[1] - p0[1])
            d = math.sqrt((dx * self.canvas_dim[0]) ** 2 + (dy * self.canvas_dim[1]) ** 2)
            needed_points_for_distance = math.ceil(d * self.min_steps_per_mm)
            for j in range(1, needed_points_for_distance + 1):
                to_add.append([p0[0] + dx * j / needed_points_for_distance,
                               p0[1] + dy * j / needed_points_for_distance])

        self._last_point_in_path = p
        self._point_queue += to_add

    def current_point(self):
        return self._point_queue[0]

    def has_next_path(self):
        return self._has_next_path and self._eof is False

    def try_next_point(self):
        self._point_queue.pop(0)

        if len(self._point_queue) == 0 and self._eof is False:
            self.__read_next_point()

        if self._has_next_path is True or self._eof is True:
            return False

        return len(self._point_queue) > 0

    def next_path(self):
        self._has_next_path = False


class PathPlotter:
    point_reached_error_threshold = abs(Constants.POINT_REACHED_ACCURACY_MM / Constants.MM_PER_DEGREE)

    def __init__(self, config: Config, mc: MotorController):
        self.mc = mc
        self.geom = Geom(config.get_anchor_distance(), config.get_canvas_offset(),
                         config.get_canvas_dim(), (1 / Constants.MM_PER_DEGREE))
        self.p0 = self.geom.get_degree(config.get_startpos_relative_to_canvas())

    def plot_path(self, pr: PathReader):
        run = True
        try:
            while run:
                point = pr.current_point()
                [left_desired_deg, right_desired_deg] = self.geom.get_degree(point)
                [left_pos, right_pos] = self.mc.get_pos()

                left_error_deg = left_desired_deg - (left_pos + self.p0[0])
                right_error_deg = right_desired_deg - (right_pos + self.p0[1])

                error = math.sqrt(left_error_deg ** 2 + right_error_deg ** 2)

                if error < self.point_reached_error_threshold:
                    # consider point reached
                    has_next = pr.try_next_point()
                    if has_next:
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
            self.mc.set_degree_per_second(0, 0)
            sys.print_exception(e)


class Plotter:

    def plot_file(self, file):
        try:
            config = Config()
            self.mc = MotorController(hub.port.B, hub.port.F)
            self.mc.preset()

            pathPlotter = PathPlotter(config, self.mc)

            pr = PathFileReader(config.get_canvas_dim())
            pr.load_path(file)

            path_count = 0

            while pr.has_next_path():
                pr.next_path()
                path_count += 1

                print('Plotting path #%s with start at %s. Moving to start...' % (path_count, pr.current_point()))
                pathPlotter.plot_path(PathListReader([pr.current_point()]))
                print('Start plotting of path')
                pathPlotter.plot_path(pr)
                print('End plotting of path')

            print('Moving back to origin')
            pathPlotter.plot_path(PathListReader([[0, 0]]))
            print('Done.')

        except Exception as e:
            print("Caught exception", e)

        # always stop motors, no matter if exception or not
        self.mc.set_degree_per_second(0, 0)


plotter = Plotter()
plotter.plot_file('/projects/path.txt')
