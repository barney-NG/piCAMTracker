#!/bin/bash

#- the monitor shall only be called once
[[ $(pgrep -c temp_monitor.sh) -gt 1 ]] && exit 1

trap go_exit 1 2 3 15

temp_dir=/run/picamtracker
temp_file=temp
sleep_time=60

mkdir -p $temp_dir

function go_exit {
  rm -f $temp_dir/$temp_file
  exit 0
}

function get_temp {
  local str=$(vcgencmd measure_temp)
  str=${str//temp=/}
  str=${str//\'C/}
  echo $str
}

while /bin/true
do
  get_temp > $temp_dir/$temp_file
  sleep $sleep_time
done
