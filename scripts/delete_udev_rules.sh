#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to station"

sudo rm   /etc/udev/rules.d/station.rules

echo " "
echo "Restarting udev"
echo ""

sudo service udev reload
sudo service udev restart

echo "finish  delete"
