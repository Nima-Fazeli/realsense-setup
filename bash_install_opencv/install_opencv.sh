#!/bin/bash

############################
## OpenCV Installer
############################

# Necessary packages
apt-get install build-essential
apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

# Setup install directory
cd $HOME/Documents
git clone https://github.com/opencv/opencv.git

# Prep cmake
cd opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..

# Build
make -j7 # runs 7 jobs in parallel

# Install libraries
sudo make install

cd $HOME

echo "Finished OpenCV Installation. \n"
