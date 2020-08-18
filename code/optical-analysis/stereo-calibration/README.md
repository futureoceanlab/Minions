## Stereo Calibration
These are software tools to calibrate the stereo imaging system:
- **detectGridPoints.cpp**: Detects center of circular grids on a calibration target

- **calibrateCameraMat.m**: Based on the detected centers, calibrate the 
stereo imaging system

An example of building the C++ code and running with the given example data

'''
mkdir build
cd build
cmake ..
make
./detectGrids 5 26 26 water-stereo
'''

The arguments are as follows: number of images, number of points along x-axis,
number of points along y-axis, and the directory that contains the data.

Running the above commands should save .txt files, which contain the detected
grid positions. Then, these .txt files can be read in the MATLAB software to 
run stereo calibration. The example parameters are provided in ''params'' 
directory.

The program requires OpenCV package

As a side note, if one is interested in using C++ to also calibrate the cameras
can take a look at calibrationRaw.cpp. This is an unclean version of the initial
efforts to calibrate the cameras.
