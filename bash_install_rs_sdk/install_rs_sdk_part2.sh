#!/bin/bash

###############################
## RealSense 2.0 SDK Installer
##############################

sudo apt-get update

sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg

modinfo uvcvideo | grep "version:"

echo "Do you see realsense in the string?\n"
echo "\n \n"
echo "Finished RealSense 2.0 SDK Installation. \n"
