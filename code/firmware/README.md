## Firmware
The current version of Minions require three separate embedded boards. Two of 
them control the cameras and one controls the LED. They communicate wirelessly,
and the LED controlling unit behaves as a server while the other two are the 
clients. Therefore, the camera controllers run on the same firmware 
(minions-cam) while the LED controller requires a separate one (minions-led).

The camera controller currently runs on the NanoPi Duo2 and the LED firmware
runs on the RPI0W. 

The 'minion-cam' firmware requires the following packages be installed: 
<a href="https://github.com/friendlyarm/WiringNP">wiringNP</a>, 
<a href="http://www.libtiff.org/">LibTIFF</a>, 
<a href="https://github.com/TheImagingSource/tiscamera">tiscamera firmware</a>.

The 'minion-led' firmware requires the following packages be installed:
<a href="http://wiringpi.com/">WiringPi</a>. 
