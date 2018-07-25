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
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all

echo "Finished PCL Installation. \n"
