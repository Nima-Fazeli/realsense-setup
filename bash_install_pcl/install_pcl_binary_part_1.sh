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
