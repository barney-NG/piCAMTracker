#!/bin/bash
#for i in 0 1 2 3 7; do echo -n "$i " && vcgencmd display_power -1 $i; done
hasmonitor=0
xset -q 2>/dev/null | grep -qi 'Monitor is On' && hasmonitor=1
if [[ $hasmonitor -gt 0 ]]
then
    /usr/bin/xrandr | /usr/bin/awk -F'[ x]+' '/\*/ { print $2,$3 }'
else
    echo "0 0"
fi
