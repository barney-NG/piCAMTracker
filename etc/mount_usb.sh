#!/bin/bash

function mount_usb {
  local mount_point=/media/pi/"$1"
  local ftype=$2
  local device=$3
  local uid=$(id -u pi)
  local gid=$(id -g pi)
  sudo mkdir -p "$mount_point"
  sudo chown pi:pi "$mount_point"
  sudo mount -t $ftype -o uid=$uid,gid=$gid $device "$mount_point"
  echo $mount_point
}

# we check just the first partition of the first drive
UUID=
for n in {1..4}
do
  device="/dev/sda$n"
  eval $(sudo blkid -o export $device)

  #echo "LABEL: $LABEL"
  #echo "UUID:  $UUID"
  #echo "TYPE:  $TYPE"
  #echo "PARTUUID:  $PARTUUID"

  if [[ -z "$UUID" ]]; then
    echo ""
    exit 0
  fi
  case "$LABEL" in
    *EFI*) ;;
    *)     break;;
  esac
done
if [[ -z "$UUID" ]]; then
  echo ""
  exit 0
fi

# setup default name
LABEL=${LABEL:=usbdrive}
# remove spaces
LABEL=${LABEL// /_}

# already mounted?
grep -q $device /etc/mtab
if [[ $? -ne 0 ]]; then
  mount_usb $LABEL $TYPE $device
else
  set -- $(grep $device /etc/mtab)
  #- \040 is converted to ' ' -- all other special characters will fail :-/
  echo ${2//\\040/ }
fi
