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
OP_DIR=/home/pi/Minions
cset shield --exec -- chrt -rr 97 $OP_DIR/imaging/build/simple-snapimage -s 15410110 -i 1 -d $OP_DIR/data
IMAGING_PID=$!
sleep 10
cset shield --exec  -- chrt -rr 98 $OP_DIR/camera/build/minions -i $IMAGING_PID -d $OP_DIR/data
