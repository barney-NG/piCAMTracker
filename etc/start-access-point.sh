#!/bin/bash

#apt -y install dnsmasq dhcpcd hostapd

parent_device="wlan0"
ssid="PICAM"
pass="Olav0101"
ipaddress="192.168.16.1/24"
dhcprange="192.168.16.50,192.168.16.200,255.255.255.0,24h"

options=$(getopt -o s: --long ssid: -- "$@")

if [[ $? -ne 0 ]]; then
  echo
  echo "usage: $0 [OPTION]"
  echo "setup wifi access point in parallel to existing client"
  echo
  echo " -s, --ssid <SSID>	Network name"
  echo
  echo "please change ip address/range and passphrase in source file: $0"
fi

eval set -- $options
while true; do
  case "$1" in
    -s|--ssid) ssid=$2; shift; shift ;;
    --) shift; break ;;
    *)  break ;;
  esac
done

#------------------------------------------------------------------------
#- exclude ap0 from dhcp service
#------------------------------------------------------------------------
grep -q 'denyinterfaces ap0' /etc/dhcpcd.conf || echo 'denyinterfaces ap0' >> /etc/dhcpcd.conf

#------------------------------------------------------------------------
#- Populate hostapd.conf
#------------------------------------------------------------------------
grep -q "ssid=${ssid}"  /etc/hostapd/hostapd.conf 2>/dev/null
if [[ $? -ne 0 ]]
then
  cat > /etc/hostapd/hostapd.conf <<EOF
interface=ap0
driver=nl80211
ssid=${ssid}
hw_mode=g
channel=7
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=${pass}
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
EOF
fi
#------------------------------------------------------------------------
#- Populate `/etc/default/hostapd`
#------------------------------------------------------------------------
grep -q '^DAEMON_CONF=' /etc/default/hostapd
if [[ $? -ne 0 ]]
then
  cat > /etc/default/hostapd << EOF
DAEMON_CONF="/etc/hostapd/hostapd.conf"
EOF
fi

#------------------------------------------------------------------------
#- Populate `/etc/dnsmasq.conf`
#------------------------------------------------------------------------
grep -q 'interface=ap0' /etc/dnsmasq.conf
if [[ $? -ne 0 ]]
then
  cat >> /etc/dnsmasq.conf << EOF

# wlan access point
except-interface=wlan0
interface=ap0
dhcp-range=${dhcprange}
EOF
fi

#------------------------------------------------------------------------
# setup ap0
#------------------------------------------------------------------------
ip a s dev ap0 > /dev/null 2>&1
if [[ $? -ne 0 ]]
then
  echo "Setting up AP: $ssid"
  macaddr=$(cat /sys/class/net/$parent_device/address)
  iw phy phy0 interface add ap0 type __ap
  ip link set ap0 address $macaddr
  ip address add ${ipaddress} dev ap0
  ip link set ap0 up
  systemctl start hostapd
  systemctl restart dnsmasq
  systemctl restart dhcpcd
fi
