#!/bin/bash

#- the background service shall only be called once
[[ $(pgrep -f -c background_service.sh) -gt 1 ]] && exit 1

while getopts ":o" opts; do
  case "$opts" in
    o) once="yes"
       ;;
  esac
done


trap go_exit SIGINT

temp_dir=/run/picamtracker
temp_file=temp
local_image_to_path=/home/pi/piCAMTracker/media/stills
local_videos_path=/home/pi/piCAMTracker/media/videos


mkdir -p $temp_dir

#- minimize wear leveling by storing images to ramdisk
#- and sync the ramdisk every 5 minutes
#- if an USB stick is attached store to the stick
function save_images {
  local usb_mount_point=$(/home/pi/piCAMTracker/etc/mount_usb.sh)
  local image_to_path=$local_image_to_path
  if [[ -n "$usb_mount_point" ]]; then
   local path_to=$usb_mount_point/media
   mkdir -p "$path_to/stills" && image_to_path=$path_to/stills
   logger "saving videos to $path_to/videos ..."
   mkdir -p "$path_to/videos" && nice rsync -aq $local_videos_path $path_to
  fi
  logger "saving images to $image_to_path..."
  nice rsync -aq $temp_dir/* "$image_to_path/."
  nice rm -f $temp_dir/*.data 2>/dev/null
}

#- exit trap
function go_exit {
  save_images
  rm -f $temp_dir/$temp_file
  exit 0
}

#- read temeparture via vcgencmd command
function get_temp {
  local str=$(vcgencmd measure_temp)
  str=${str//temp=/}
  str=${str//\'C/}
  echo $str
}

if [[ "$once" == "yes" ]]
then
  save_images
  exit 0
fi

#- make a one minute loop
loop=1
while /bin/true
do
  get_temp > $temp_dir/$temp_file
  #- save images every 5 minutes
  [[ $((loop++ % 5)) -eq 0 ]] && save_images
  sleep 60
done
