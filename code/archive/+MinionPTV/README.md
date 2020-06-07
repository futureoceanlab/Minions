# MinionPTV
This is a MATLAB package for underwater particle tracking velocimetry (PTV) used in Minions project. 

There availble modules as follows:

- **calibration**: A module for calibrating a stereo imaging system. It uses a packaged C++ code, which uses OpenCV, to detect circular grids.

- **simulation**: The simulation data is essential in debugging PTV and predicting the performance in real deployment. The simulation also saves ground truth data for later comparision on evalatuion module. 

- **track**: Main module of the package. It closely follows the ptv algorithm developed in software used for underwater PTV by Simoncelli ([repository](https://github.com/s-simoncelli/ptv/blob/master/README.md)). This software was used in

> Simoncelli et al. 2019. A low-cost underwater particle tracking velocimetry system for measuring in-situ particle flux and sedimentation rate in low-turbulence environments. Limnology and Oceanography: Methods DOI:10.1002/lom3.10341

- **evaluation**: module for evaluating the performance of the PTV algorithm. The module compares the algorithm performance compared against the ground truth provided in simulation data. 
