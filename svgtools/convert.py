#!/usr/bin/env python

import argparse
import math
from os import getcwd, path as os_path

from svgpathtools import svg2paths, Path, Line, disvg

parser = argparse.ArgumentParser(description='Convert a svg to wallplotter format')
parser.add_argument('file', type=str, help='svg file to convert')
parser.add_argument('out', type=str, help='output filename without suffix')
# 4 decimals = 1mm precision at 100cm canvas size
parser.add_argument('-p', '--precision', type=int, default=4, help='how many decimals. Default: 4')

args = parser.parse_args()

discontinued_paths, attributes = svg2paths(args.file)
paths = [continued_path for disc_path in discontinued_paths for continued_path in disc_path.continuous_subpaths()]
n_digits = args.precision

result = []
min_x = math.inf
max_x = -math.inf
min_y = math.inf
max_y = -math.inf

for path in paths:
    steps = math.ceil(path.length() / 5)

    if steps == 0:
        continue

    path_result = []
    result.append(path_result)

    last_slope = math.inf

    for p in range(0, steps + 1):
        coords = path.point(p / steps)
        x = coords.real
        y = coords.imag

        should_replace = False
        if len(path_result) > 0:
            last_added = path_result[-1]
            slope = math.atan2(y - last_added[1], x - last_added[0])
            should_replace = math.isclose(slope, last_slope, rel_tol=1e-3)
            if should_replace is False:
                last_slope = slope

        min_x = min(min_x, x)
        max_x = max(max_x, x)
        min_y = min(min_y, y)
        max_y = max(max_y, y)

        if should_replace:
            path_result[-1] = [x, y]
        else:
            path_result.append([x, y])

# rescale to [0, 1]
for path in result:
    for p in path:
        p[0] = round((p[0] - min_x) / max_x, n_digits)
        p[1] = round((p[1] - min_y) / max_y, n_digits)

# sort paths by distance between end and start
result_sorted = []
while len(result) > 0:
    if len(result_sorted) == 0:
        result_sorted.append(result.pop())
    else:
        last_added = result_sorted[-1]


        def distance_to_last(p):
            return math.sqrt((last_added[-1][0] - p[0][0]) ** 2 + (last_added[-1][1] - p[0][1]) ** 2)


        closest_path = sorted(result, key=distance_to_last)[0]

        result_sorted.append(closest_path)
        result.remove(closest_path)

# write data as python lists
with open(args.out + '.py', 'w') as filehandle:
    filehandle.write("paths = [\n")
    for path in result_sorted:
        filehandle.write("    [\n")
        filehandle.writelines("        [%s, %s],\n" % (point[0], point[1]) for point in path)
        filehandle.write("    ],\n")
    filehandle.write("]\n")

with open(args.out + '.txt', 'w') as filehandle:
    for path in result_sorted:
        filehandle.writelines("%s,%s\n" % (point[0], point[1]) for point in path)
        filehandle.write("\n")

# create preview svg
preview = []
for path in result_sorted:
    preview_p = Path()
    preview.append(preview_p)
    p0 = path[0]
    for p1 in path[1:]:
        preview_p.append(Line(complex(p0[0], p0[1]), complex(p1[0], p1[1])))
        p0 = p1

disvg(paths=preview, filename=os_path.join(getcwd(), args.out + '.svg'), margin_size=0)
