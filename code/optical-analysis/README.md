## Optical Analysis
This directory contains codes that pertain to analysis of empirical
measurements for the optics.

### Camera Foucs
This is a tool to assist with focusing the imaging system while doing
a camera resolution test. It contains a C++ code that measures and outputs
contrast at the ROI at the center of the image.

### Camera Resolution Analysis
Upon taking images of the Siemens star, we analyze the contrast at different
radial distance from the center. The closer to the center, the lower the 
resolution. Ideally, we would like to use a lens that allows for 20% contrast
at 20um/px resolution. However, no such lens could be found, so we opted
the best performing lens. Comparisons among the lenses as well as the 
performance in air and in water are made.

### LED FWHM Analysis
We want to fill our imaging target volume, but we do not want to waste energy
by illuminating irrelevant volume in the water. For that, we need to find a 
combination of the LED and the light optics that give us the apporpriate 
full width half maximum in the water.

### Stereo Calibration
Upon determining the single camera optics (i.e., imaging sensor and the lens),
we need to set up a stereo pair. We have designed the ideal mechanical setup
for the stereo pair, but in the real life, there are errors. Therefore, we need
to calibrate. The tools in here are meant to help with the calibration process.