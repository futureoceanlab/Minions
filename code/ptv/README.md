## Particle Trackinv Velocimetry (PTV)

This repository contains MATLAB codes for simulation and evaluation of 
particle tracking velocimetry (PTV) applied for analysis of settling motion 
of the marine snow in the ocean. 

There are three major codes that can be used:
- **simulate.m** : generates images of simulated marine snow particles (spheres
and cylinders) seen by the left and right cameras. This requires a 
<a href="">calibrated</a>
or a simulated 
<a href="https://www.mathworks.com/help/vision/ref/stereoparameters.html">
stereo camera parameter.</a> A simulated parameter based on the design in 
my thesis can be seen in ``data/stereocamera_3/params.mat'', created using
**createStereoParams.m**. The simulation is based on the data provided in

> A. Mcdonnell, Marine particle dynamics: sinking velocities, size distributions,
fluxes, and microbial degradation rates. PhD thesis, 08 2011.

- **evaluate.m**: performs PTV on the simulated image directory. The script
further compares the PTV results to groundtruth data from the simulation.

- **runPTV.m**: a function to perform PTV on a given set of images. It outputs
a table of tracks. This code closely follows the package 
provided in <a href="https://github.com/s-simoncelli/ptv">Underwater PTV</a>
with some modifications.
> Simoncelli et al. 2019. A low-cost underwater particle tracking velocimetry 
system for measuring in-situ particle flux and sedimentation rate in 
low-turbulence environments. Limnology and Oceanography: 
Methods DOI:10.1002/lom3.10341