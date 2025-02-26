#!/bin/bash

echo "remap the device serial port(ttyUSBX) to station"
echo "station usb connection as /dev/station , check it using the command : ls -l /dev | grep ttyUSB"

sudo cp `rospack find measurement_station`/scripts/station.rules  /etc/udev/rules.d

echo " "
echo "Restarting udev"
echo ""

sudo service udev reload
sudo service udev restart
sudo udevadm control --reload && sudo udevadm trigger

echo "finish "
