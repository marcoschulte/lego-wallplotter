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

paths, attributes = svg2paths(args.file)
n_digits = args.precision

result = []
min_x = math.inf
max_x = -math.inf
min_y = math.inf
max_y = -math.inf

for path in paths:
    length = path.length()

    path_result = []
    result.append(path_result)

    for p in range(0, math.floor(length + 1)):
        coords = path.point(p / length)
        x = coords.real
        y = coords.imag

        min_x = min(min_x, x)
        max_x = max(max_x, x)
        min_y = min(min_y, y)
        max_y = max(max_y, y)

        path_result.append([x, y])

# rescale to [0, 1]
for path in result:
    for p in path:
        p[0] = round((p[0] - min_x) / max_x, n_digits)
        p[1] = round((p[1] - min_y) / max_y, n_digits)

# write data
with open(args.out + '.py', 'w') as filehandle:
    filehandle.write("paths = [\n")
    for path in result:
        filehandle.write("    [\n")
        filehandle.writelines("        [%s, %s]\n" % (point[0], point[1]) for point in path)
        filehandle.write("    ],\n")
    filehandle.write("]\n")

# create preview svg
preview = []
for path in result:
    preview_p = Path()
    preview.append(preview_p)
    p0 = path[0]
    for p1 in path[1:]:
        preview_p.append(Line(complex(p0[0], p0[1]), complex(p1[0], p1[1])))
        p0 = p1

disvg(paths=preview, filename=os_path.join(getcwd(), args.out + '.svg'), margin_size=0)
