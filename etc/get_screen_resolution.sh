#!/bin/bash
/usr/bin/xrandr | /usr/bin/awk -F'[ x]+' '/\*/ { print $2,$3 }'
