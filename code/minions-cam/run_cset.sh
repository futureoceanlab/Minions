#!/bin/bash

cpu_freq -s 1200000

#Core_c=2
#Core_i=3
#Core=6
#CoreMask=`echo "16 o 2 $Core ^ p" | dc`
#echo $CoreMask

cset shield --cpu=1-3 --kthread=on
#cset set --cpu=$Core_c --set=c_set
#cset set --cpu=$Core_i --set=i_set

cset shield --exec -- chrt -rr 97 /home/pi/imaging/build/simple-snapimage 0 1 &
sleep 10
cset shield --exec  -- chrt -rr 98 /home/pi/camera/build/minions
