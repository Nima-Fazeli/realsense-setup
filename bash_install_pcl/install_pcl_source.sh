#!/bin/bash

#############################################
# Install VTK Dependency
cd $HOME/Documents
git clone git://vtk.org/VTK.git
cd VTK

git fetch origin
git rebase origin/master

mkdir VTK-Release-build
cd VTK-Release-build
cmake -DCMAKE_BUILD_TYPE:STRING=Release ../VTK

make -j4

cd $HOME

#############################################
# Install PCL Binary
PCL_VER="1.8.1"

# Go to download directory
cd $HOME/Downloads/

# Unzip the tar
tar xvzf pcl-pcl-$PCL_VER.tar.gz

# Enter the folder and build
cd pcl-pcl-$PCL_VER && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j2
sudo make -j2 install

echo "Finished PCL Installation. \n"
