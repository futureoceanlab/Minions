### Light Simulation 
We need to understand how much light power is required to illuminate the 
particles in the region of interest. We settled on having two LEDs that are 
symmetrically placed abount the x-z plane. 

First the intensity profile of the particular combination of optics (led and
reflector/lens) needs to be estiamted. This is done in **radiantPower.m**.

Then, we move the locations of the light sources to see which configuration
yields a reasonable detection error as well as illumination. In particular,
we choose to change the angle and the distance of the light source relative 
to the target volume. This is simulated with nine particles that are dispersed
within the target volume in **estimateIntensity.m**.