    # copy test file
    rshell -p /dev/cu.LEGOHubA8E2C19D07F2-Ser
    cp ../svgtools/out/cube.txt /wallplotter/

    # plot test file
    ./start.sh
    plotter.plot_file('/wallplotter/cube.txt')