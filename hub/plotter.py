import math
import sys
import time

import hub


class Constants:
    # which ports are the motors connected to
    MOTOR_LEFT = hub.port.B
    MOTOR_RIGHT = hub.port.A
    MOTOR_PEN = hub.port.C

    # rope length change in mm per degree rotation of the motor
    # number should be:
    # * positive, if motor position value increases when retracting rope,
    # * negative, if motor position value increases when rope extends
    MM_PER_DEGREE_LEFT = 3760 / 137816
    MM_PER_DEGREE_RIGHT = -3760 / 137816

    # absolute motor position when drawing / not drawing
    PEN_UP = 0  # not drawing
    PEN_DOWN = 180  # drawing

    POWER_MAX_PERCENTAGE = 1.0  # use only XX% of available motor power
    POWER_PER_DEGREE_PER_SECOND = 1 / 9.3  # factor to convert from desired deg/s to power that needs to be applied
    MAX_DEG_PER_S = 100 / POWER_PER_DEGREE_PER_SECOND * POWER_MAX_PERCENTAGE
    POINT_REACHED_ACCURACY_MM = 1  # how close (in mm) do we need to be at a point to consider it reached
    MAGIC_MOTOR_MODE = [(1, 0), (2, 2), (3, 1), (0, 0)]


class Config:
    def get_startpos_relative_to_canvas(self):
        """
        :return: The starting position of the robot on the canvas. Usually [0,0]
        """
        return [0, 0]

    def get_canvas_dim(self):
        """
        :return: The canvas' dimension (width, height) in mm
        This is how big the plot will be. Also the minimum size of
        the sheet you are printing on, if you don't want to ruin your wall
        """
        return [500, 321]

    def get_anchor_distance(self):
        """
        :return: Distance between the two rope anchors in mm.
        """
        return 790

    def get_canvas_offset(self):
        """
        :return: Translation of the canvas coordinate system relative to the left anchor
        ELI5: How much do you go right and down from the top left anchor to
        the top left edge of your sheet
        """
        return self.calc_canvas_offset_from_rope_length(530, 820)

    def calc_canvas_offset_from_rope_length(self, left_mm, right_mm):
        """
        :return: calculate the canvas offset from measured rope lengths
        """
        a = (math.pi / 2) - math.acos((-right_mm ** 2 + left_mm ** 2 + self.get_anchor_distance() ** 2) / (
                2 * left_mm * self.get_anchor_distance()))
        offset_left_mm = math.sin(a) * left_mm
        offset_right_mm = math.cos(a) * left_mm
        return [offset_left_mm, offset_right_mm]


class Geom:
    def __init__(self, d, offset, canvas_dim, degree_per_mm_left, degree_per_mm_right):
        """
        Calculates motor positions for coordinates
        :param d: distance d between two anchors
        :param offset: offset [x, y] of the canvas relative to the left anchor
        :param canvas_dim: dimensions of the canvas
        :param degree_per_mm_left: how many degrees doos the left motors need to turn for one mm extension of the rope
        :param degree_per_mm_right: how many degrees does the right motors need to turn for one mm extension of the rope
        """
        self.d = d
        self.offset = offset
        self.canvas_dim = canvas_dim
        self.degree_per_mm_left = degree_per_mm_left
        self.degree_per_mm_right = degree_per_mm_right

    def get_degree(self, point):
        x = point[0]
        y = point[1]
        l = math.sqrt(
            (x * self.canvas_dim[0] + self.offset[0]) ** 2 + (
                    y * self.canvas_dim[1] + self.offset[1]) ** 2) * self.degree_per_mm_left
        r = math.sqrt((self.d - self.offset[0] - x * self.canvas_dim[0]) ** 2 + (
                y * self.canvas_dim[1] + self.offset[1]) ** 2) * self.degree_per_mm_right
        return [l, r]


class PenController:
    speed = 30

    def __init__(self, port):
        self.port = port
        self.port.motor.mode(Constants.MAGIC_MOTOR_MODE)

    def __move_to_pos_if_not_there(self, desired_pos):
        dif = desired_pos - self.port.motor.get()[2]
        if dif < -180:
            dif += 360
        if dif > 180:
            dif -= 360
        if abs(dif) > 5:
            self.port.motor.run_for_degrees(abs(dif), round(math.copysign(self.speed, dif)))
            time.sleep(1)

    def start_drawing(self):
        self.__move_to_pos_if_not_there(Constants.PEN_DOWN)

    def stop_drawing(self):
        self.__move_to_pos_if_not_there(Constants.PEN_UP)


class MotorController:
    def __init__(self, port_left, port_right):
        self.port_left = port_left
        self.port_right = port_right
        self.start_pos_left = 0
        self.start_pos_right = 0
        self.port_left.motor.mode(Constants.MAGIC_MOTOR_MODE)
        self.port_right.motor.mode(Constants.MAGIC_MOTOR_MODE)

    def preset(self):
        [self.start_pos_left, self.start_pos_right] = self.get_pos()

    def get_pos(self):
        return [self.port_left.motor.get()[1] - self.start_pos_left,
                self.port_right.motor.get()[1] - self.start_pos_right]

    def set_degree_per_second(self, left, right):
        self.port_left.pwm(round(left * Constants.POWER_PER_DEGREE_PER_SECOND))
        self.port_right.pwm(round(right * Constants.POWER_PER_DEGREE_PER_SECOND))

    def brake(self):
        self.set_degree_per_second(0, 0)
        self.port_left.motor.brake()
        self.port_right.motor.brake()


class PathReader:
    def try_next_point(self):
        pass

    def current_point(self):
        pass

    def progress(self):
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


class InterpolatingPathFileReader(PathReader):
    min_steps_per_mm = 1

    def __init__(self, canvas_dim):
        self.canvas_dim = canvas_dim
        self.reset()

    def reset(self):
        self._point_queue = []
        self._last_point_in_path = None
        self._has_next_path = False
        self._eof = False
        self._progress = 0
        self._progress_max = 0

    def load_path(self, file):
        self.reset()

        fh = open(file, 'r')
        fh.seek(0, 2)  # seek to end of file
        self._progress_max = fh.tell()
        fh.close()

        self._fh = open(file, 'r')
        self._has_next_path = True
        self.__read_next_point()

    def progress(self) -> (int, int):
        return self._progress, self._progress_max

    def __read_next_point(self):
        line = False
        last_tell = self._fh.tell()

        while line is False or len(line) == 0:
            line = self._fh.readline().strip()
            new_tell = self._fh.tell()
            self._progress = new_tell
            if last_tell == new_tell:
                # reached eof
                self._eof = True
                self._fh.close()
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

            if needed_points_for_distance == 0:
                to_add.append(p)
            else:
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
    point_reached_error_threshold = abs(Constants.POINT_REACHED_ACCURACY_MM / Constants.MM_PER_DEGREE_LEFT)

    def __init__(self, config: Config, mc: MotorController):
        self.mc = mc
        self.geom = Geom(config.get_anchor_distance(), config.get_canvas_offset(),
                         config.get_canvas_dim(), (1 / Constants.MM_PER_DEGREE_LEFT),
                         (1 / Constants.MM_PER_DEGREE_RIGHT))
        self.p0 = self.geom.get_degree(config.get_startpos_relative_to_canvas())

    def plot_path(self, pr: PathReader, progress_callback=None):
        run = True
        last_desired_degree = None

        while run:
            point = pr.current_point()
            [left_desired_deg, right_desired_deg] = self.geom.get_degree(point)
            [left_pos_rel, right_pos_rel] = self.mc.get_pos()
            [left_pos, right_pos] = [left_pos_rel + self.p0[0], right_pos_rel + self.p0[1]]

            left_error_deg = left_desired_deg - left_pos
            right_error_deg = right_desired_deg - right_pos

            error = math.sqrt(left_error_deg ** 2 + right_error_deg ** 2)

            reached = error < self.point_reached_error_threshold
            if not reached and last_desired_degree is not None:
                # test whether we already passed the point but missed the circle we consider reached:
                # vector from last point to current point
                v_lp_p = [left_desired_deg - last_desired_degree[0], right_desired_deg - last_desired_degree[1]]
                # vector from current point to plotters position
                v_pos_p = [left_pos - left_desired_deg, right_pos - right_desired_deg]
                scalar = v_lp_p[0] * v_pos_p[0] + v_lp_p[1] * v_pos_p[1]
                reached = scalar >= 0

            if reached:
                # consider point reached
                has_next = pr.try_next_point()
                if has_next:
                    last_desired_degree = [left_desired_deg, right_desired_deg]
                    if progress_callback is not None:
                        progress_callback(pr.progress())
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


class ProgressReporter:
    def __init__(self):
        self._num_path = 1
        self._percentage = 0
        self._start_millis = 0

    def update_num_path(self, num_path):
        self._num_path = num_path

    def update_percentage(self, percentage):
        self._percentage = percentage

    def start(self):
        self._start_millis = time.ticks_ms()

    def print(self):
        num_max_hash = 20
        num_hash = math.floor(self._percentage * num_max_hash)
        num_blank = num_max_hash - num_hash
        progress_bar = '#' * num_hash + ' ' * num_blank

        now = time.ticks_ms()
        elapsed = now - self._start_millis

        line = '[{}] {:.0%} ({}s) - Path #{}, Battery: {}%'.format(
            progress_bar, self._percentage, round(elapsed / 1000), self._num_path,
            hub.battery.capacity_left())
        print('\r\033[K{}'.format(line), end='')


class Plotter:
    def __init__(self):
        self.config = Config()
        self.pc = PenController(Constants.MOTOR_PEN)
        self.mc = MotorController(Constants.MOTOR_LEFT, Constants.MOTOR_RIGHT)
        self.mc.preset()
        self.path_plotter = PathPlotter(self.config, self.mc)
        self.progress_report = ProgressReporter()

    def plot_file(self, file):
        try:
            self.pc.stop_drawing()
            self.progress_report.start()

            pr = InterpolatingPathFileReader(self.config.get_canvas_dim())
            pr.load_path(file)

            num_path = 0

            while pr.has_next_path():
                pr.next_path()
                num_path += 1
                self.progress_report.update_num_path(num_path)
                self.progress_report.print()  # print progress only if not moving as it causes delay in control loop

                # move to start of path
                self.path_plotter.plot_path(PathListReader([pr.current_point()]))
                self.mc.brake()
                # move to start a second time to compensate possible drift while stopping
                self.path_plotter.plot_path(PathListReader([pr.current_point()]))
                self.mc.brake()

                # plot path
                self.pc.start_drawing()
                self.path_plotter.plot_path(pr, self.progress_callback)

                # path finished
                self.mc.brake()
                self.pc.stop_drawing()

            print('\nMoving back to origin')
            self.path_plotter.plot_path(PathListReader([[0, 0]]))
            print('Done.')

        except BaseException as e:
            self.mc.brake()
            print("\nCaught exception", e)
            sys.print_exception(e)
            self.pc.stop_drawing()
            print("Motor pos", self.mc.get_pos())

        # stop motors
        self.mc.brake()

    def progress_callback(self, progress: (int, int)):
        cur, total = progress
        self.progress_report.update_percentage(cur / total)


plotter = Plotter()
# plotter.plot_file('/wallplotter/cube.txt')
