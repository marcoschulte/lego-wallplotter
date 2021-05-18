# Install requirements

    $ pip install -r requirements.txt

# Prepare SVGs

The conversion script does not handle svg transforms correctly (actually does not handle them at all).

Therefore, the svg cannot contain transforms and they must be applied to the path's coordinates before converting the svgs to the plotters format. I did this successfully with both Inkscape or Illustrator.

## Inkscape

Install plugin https://github.com/Klowner/inkscape-applytransforms

* Open SVG
* Select all
* Path -> Object to path
* Extension -> Modify Paths -> Apply transform

## Illustrator

* Select object
* Transform -> x: 0, y: 0
* "Export as" + use artboards -> svg

# Convert

SVGs can be converted to the plotter's file format with the following command

    python convert.py some.svg /path/to/output/folder

The script has two optional parameters

* `--sampling`: A factor which determines how many points are created when rasterising a path. Can be increased if arcs aren't smooth, decreased to decrease file size.
* `--precision`: Number of decimal places in output file. Depending on the canvas size might make sense to increase.

The script writes three files:

* A `.txt` this is the file that can be plotted with the robot
* A `.svg` this is a preview of what will be plotted. Good to verify everything is converted as expected
* A `.py` This file contains the same data as the `.txt` just as a python list. Sometimes handy for debugging stuff etc