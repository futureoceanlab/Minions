## Tool to focus a camera

### Building
In order to build the tool, open a terminal, enter the sample's directory. Then enter
```
mkdir build
cd build 
cmake ..
make
./singleFocus -s 12345678 -x 2592 -y 1944 -r 500
```
The input arguments are serial number, image frame width, height and region of interest length

### Required packages
To run the code, one needs OpenCV and 
<a href="https://github.com/TheImagingSource/tiscamera">Tiscamera</a> packages.
The wrapper for C++ is provided in 'tcamcamera.cpp'. 

### Note
The resolution target that was used for focusing is a 
<a href="https://www.thorlabs.com/thorproduct.cfm?partnumber=R1L1S2P">1''x1'' 
Siemens star</a> from Thorlabs Inc. Given the desired resolution of 20um/px,
the corresponding star fills 500x500 px square ROI at the center. We apply sobel
filter and find the magnitudes of the contrast this ROI. We adjust either
the lens or the micrometer so that the contrast value reaches the maximum.
