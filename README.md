# Minions
This repository contains software and hardware assets for the Minions 
stereo imaging system project. The design and evaluation based on these
assets are thoroughly described in 
<a href="https://www.dropbox.com/s/i847r4oa8v05839/junsuj-ms-2020.pdf?dl=0">Junsu's thesis</a>.

## Circuit
This directory contains the circuit related assets: schematic, board layout 
and the gerber file. They were used to develop and evaluate the necessary
components to communicate and control peripheral electronics of the 
Minion stereo camera components. EAGLE software is required.

## Code
The software components of the project can be broken down into four major
parts:
- **PTV**: We devised a simulation of the settling marine snow. We also 
apply PTV to track those particles and evaluated the performance of the PTV. 
- **Firmware**: Firmware to control cameras and LED during deployment.
- **Optical Simulation**: Simulation tools to evaluate the designs of the stereo
optical system.
- **Optical Analysis**: Analysis of empirical measurements of the imaging 
system and the lighting systems.

More detailed explanations of the code can be found in each of the sub-directories
and in the code itself.

## Mechanical
The mechanical component of the project remains to be done. The assets will
be added as the project progresses.