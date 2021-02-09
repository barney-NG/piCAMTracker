#!/bin/bash
hasmonitor=0
xset -q 2>/dev/null | grep -qi 'Monitor is On' && hasmonitor=1
if [[ $hasmonitor -gt 0 ]]
then
    /usr/bin/xrandr | /usr/bin/awk -F'[ x]+' '/\*/ { print $2,$3 }'
else
    echo "0 0"
fi
