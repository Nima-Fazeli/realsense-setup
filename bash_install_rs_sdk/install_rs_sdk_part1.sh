#!/bin/bash

# Add Intel server to the list of repositories:
echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | sudo tee /etc/apt/sources.list.d/realsense-public.list
apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE
