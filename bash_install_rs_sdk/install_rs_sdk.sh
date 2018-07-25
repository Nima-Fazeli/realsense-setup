#!/bin/bash

###############################
## RealSense 2.0 SDK Installer
##############################

# Add Intel server to the list of repositories:
echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | sudo tee /etc/apt/sources.list.d/realsense-public.list
apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE

sudo apt-get update

sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg

modinfo uvcvideo | grep "version:"

echo "Do you see realsense in the string?\n"
echo "\n \n"
echo "Finished RealSense 2.0 SDK Installation. \n"
