# Realsense Camera D Series Setup & NVIDIA GPU Driver Setup:

This repository contains my findings in setting up a Realsense D Series camera from scratch. The idea is to refer to this guide later down the line to redo things efficiently. Things you'll need:

1. Ubuntu 16.04
2. Kernel 4.13
3. USB3 (very important)
4. Software libraries
5. ROS repo for realsense-ros

An important reference is the [ROS Wrapper for Real Sense Devices Github page](https://github.com/intel-ros/realsense/#installation-instructions). 

## Pre-requisits

### Operating system and Kernel
I have only been successful in getting the realsense to work reliably on `Ubuntu 16.04`, it may work on other versions (it is supposed to work on 14.04) but not sure right now. Further, the most stable version of the realsense package requires `kernel 4.13`.

Here are two methods to get Kernel 4.13 installed (second one may be easier):

#### Method 1:

Update and prepare the OS:
```
sudo apt update
sudo apt upgrade -y
sudo reboot
poweroff
```
Check your current kernel installation:
```
uname -msr
```
You'll probably get `Linux 4.4.0-78-generic x86_64`, which means you're running kernel 4.4. Now make a new directory
```
sudo mkdir -p ~/4.13
cd ~/4.13
```
The go to [ubuntu kernels](http://kernel.ubuntu.com/%7Ekernel-ppa/mainline/), and find the generic version of the kernel you want. You can use widgets like:
```
wget http://kernel.ubuntu.com/~kernel-ppa/mainline/v4.11.2/linux-headers-4.11.2-041102_4.11.2-041102.201705201036_all.deb
wget http://kernel.ubuntu.com/~kernel-ppa/mainline/v4.11.2/linux-headers-4.11.2-041102-generic_4.11.2-041102.201705201036_amd64.deb
wget http://kernel.ubuntu.com/~kernel-ppa/mainline/v4.11.2/linux-image-4.11.2-041102-generic_4.11.2-041102.201705201036_amd64.deb
```
Then run:
```
dpkg -i *.deb
sudo update-grub
sudo reboot
```
Then you can check to make sure it changed, if it didnt then repeat, but run this:
```
dpkg -i *.deb
sudo update-grub
sudo poweroff
```
The poweroff is important because it seems reboot doesnt always trigger the necessary condition to change kernel.

You can also download the kernel and place it in the folder you made and run it as an installer, then repeat the previous steps.

#### Method 2:
Install `ukuu` by:
```
sudo add-apt-repository ppa:teejee2008/ppa
sudo apt-get update && sudo apt-get install ukuu
```
Run ukuu as an application, then browse to find the kernel you want and install it. Once done, run:
```
sudo poweroff
```

At this point you have Ubuntu 16.04, ROS, and kernel 4.13, you're ready to move on.

### Software libraries

The most important library is `librealsense2`, go to the [librealsense2 github page](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages), and follow the instructions, you may need to install some additionally libraries, I'll try to make a list of the things you may need:
```
sudo apt-get update 
```
I found I did need to do something with nodelets (installing them) but it may not be necessary. 

IMPORTANT: I found that I needed to reboot several times during this step, so if something doesnt work, or you need to do some extra work, reboot after. There is no harm in it. Once you're done with this step do try to reboot again as well.

### USB3 Check
You can check what usb port is USB3 by running:
```
lsusb -t
```
Then take a look at the results, anything with speed of 5000 is USB3, it also has `xhci` in the name just in case. Make sure to connect cameras to USB3 ports.


### Camera firmware update
We need to make sure the camera has the latest firmware. To do this:

1. Go to [RealSense Firmware Website](https://realsense.intel.com/intel-realsense-downloads/#firmware). Download the latest firmware available.
2. Go to [RealSense Linux Firmware Guide](https://www.intel.com/content/www/us/en/support/articles/000028171/emerging-technologies/intel-realsense-technology.html), then follow the instructions in the PDF.

IMPORTANT: When running step 2, you need to run the camera upgrade line of code with `sudo`, otherwise you may receive an error regarding usb permissions and something about switching.

IMPORTANT: Once done with this step it is worth rebooting.


## ROS RealSense Wrapper Package and Installation:

Once done with the above steps, go to the [ROS RealSense Wrapper Github page](https://github.com/intel-ros/realsense/#installation-instructions). In theory with the above steps, you have fullfilled the step 1 and requirements and you should only need to follow on from step 2. 

IMPORTANT: Once done with this step it is worth rebooting, you may find you need to reboot several times during the steps here, that is to be expected.

IMPORTANT: I first cloned this package, then noticed some dependencies and issues, and after fixing each, I found it was necessary to keep running `catkin_make` as:
```
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
```

At this point hopefully you can run 
```
roslaunch realsense2_camera rs_rgbd.launch
```
To do the visualization, you need to manually call `rviz`:
```
rviz
```
In `rviz`, you need to first specify `Global Options/Fixed Frame` as the `camera_link`, then add `pointcloud2` topic.


# NVIDIA Graphic Card Setup on Ubuntu

If you have a fresh machine with a fancy GPU, such as a 1080TI, here are the steps to set up a new graphics card. For reference, I used (this page)[https://blog.nelsonliu.me/2017/04/29/installing-and-updating-gtx-1080-ti-cuda-drivers-on-ubuntu/]

## 1. Install Ubuntu

Create a bootable Ubuntu installation USB, for example [how to do it on Ubuntu](https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-ubuntu#0).

Then in the bios setup the boot to prioritze the USB and then install Ubuntu.

## 2. Configure Integrated Graphics Card

The Ubuntu installation does not come with drivers to support the graphics card, so booting up will results in a blank screen. The solution is to configure the bios to use the integrated graphics card. In this instance you enable both iGPU and the GPU.

## 3. Driver Installation

The easy was is to do this:
```
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt-get update
```
The you want to go to the (Binary Driver How To for NVIDIA)[https://help.ubuntu.com/community/BinaryDriverHowto/Nvidia], and check the latest driver number, as of the date of this guide it was 390, then do this:
```
sudo apt-get install nvidia-390
```
This will install the drivers. Cool, now you need to reboot, I prefer to power off:
```
poweroff
```
Then boot back into Ubuntu and then you can run the follow:
```
nvidia-settings
nvidia-smi
```
The first will show you the current settings and your graphics card, the second will just provide you with a quick summary of the GPU status, it will also verify CUDA (I think). At this point you're good to go.

## 4. Updating your drivers:

In case you need to update your drivers you first need to remove the old one:
```
sudo apt-get purge nvidia*
```
Then follow step one again.

