#!/usr/bin/env python

import math


class PathReader:
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
        self.read_next_point()

    def read_next_point(self):
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

        p = [float(x) for x in line.split(",", maxsplit=2)]

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

    def has_next_point(self):
        return len(self._point_queue) > 0 and self._has_next_path is False and self._eof is False

    def has_next_path(self):
        return self._has_next_path and self._eof is False

    def next_point(self):
        p = self._point_queue.pop(0)
        if len(self._point_queue) == 0 and self._eof is False:
            self.read_next_point()
        return p

    def next_path(self):
        self._has_next_path = False


p = PathReader([2, 2])
p.load_path('out/yourtext.txt')

idx = 0

while p.has_next_path():
    p.next_path()

    idx += 1

    print('Start of path', idx)
    while p.has_next_point():
        print('    ' + str(p.next_point()))
    print('End of path\n')
