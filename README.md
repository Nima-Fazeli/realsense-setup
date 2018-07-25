# Realsense Camera D Series Setup & NVIDIA GPU Driver Setup:

# Realsense Camera D Series Setup:

Here are some helpful steps to set up a Realsense D Series camera from scratch. The guide should be self contained and not too much knowledge is required to get things working, but if you have suggestions for improvements, please contact me. Things you'll need:

1. Ubuntu 16.04
2. Kernel 4.13
3. USB3 (very important)
4. Software libraries
5. ROS repo for realsense-ros

An important reference is the [ROS Wrapper for Real Sense Devices Github page](https://github.com/intel-ros/realsense/#installation-instructions). 

## Pre-requisits

### Operating system and Kernel
I have only been successful in getting the realsense to work reliably on `Ubuntu 16.04`, it may work on other versions (it is supposed to work on 14.04) but not sure right now. Further, the most stable version of the realsense package requires `kernel 4.13`, I use the latest `4.13`. Things should, at least in theory, also work on `4.10`.

Here are two methods to get Kernel 4.13 installed:

#### Method 1 (Easier):

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
You'll probably get `Linux 4.4.0-78-generic x86_64`, which means you're running kernel 4.4.

Install `ukuu` by:
```
sudo add-apt-repository ppa:teejee2008/ppa
sudo apt-get update && sudo apt-get install ukuu
```
Run ukuu as an application, then browse to find the kernel you want and install it. Once done, run:
```
sudo poweroff
```
The poweroff is important because you need to power cycle and a reboot does not always do the job. Now check your current kernel installation:
```
uname -msr
```
At this point you have Ubuntu 16.04, ROS, and kernel 4.13, you're ready to move on. If the kernel did not update and you have a newer kernel, you need to downgrade, follow the steps below:

If you need to downgrade your kernel instead of upgrade, you can follow the instruction in this [how to](https://www.makeuseof.com/tag/upgrade-kernel-ukuu-ubuntu/), but in short, you need to install the desired Kernel from ukuu, then do this:
```
sudo nano /etc/default/grub
```
Comment out the `GRUB_HIDDEN_TIMEOUT` and `GRUB_HIDDEN_TIMEOUT_QUITE` lines using `#`, then run:
```
sudo grub-mkconfig -o /boot/grub/grub.cfg
```
Then when you reboot, in the boot selection screen in Ubuntu, select `Advanced options for Ubuntu`, then select the correct kernel you want to boot up in.

You can also clean up after and remove the kernels you don't want. 


#### Method 2 (Manual and a little involved):

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

### Software libraries

You'll need:

1. OpenCV
2. PCL
3. librealsense2

I have written shell scripts for installation of these libraries, you can use:
```
sudo chmod +x bash_install_opencv/install_opencv.sh && sudo bash_install_opencv/install_opencv.sh
sudo chmod +x bash_install_pcl/install_pcl_binary_part_1.sh && sudo bash_install_opencv/install_pcl_binary_part_1.sh
sudo chmod +x bash_install_pcl/install_pcl_binary_part_2.sh && sudo bash_install_opencv/install_pcl_binary_part_2.sh
sudo chmod +x bash_install_rs_sdk/install_rs_sdk_part1.sh && sudo bash_install_opencv/install_rs_sdk_part1.sh
sudo chmod +x bash_install_rs_sdk/install_rs_sdk_part2.sh && sudo bash_install_opencv/install_rs_sdk_part2.sh
```

For more details, there is a manual way of doing it, I have a guide for that in this repository also.

For `librealsense2`, go to the [librealsense2 github page](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages) as a reference.

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

TLDR: In short you need to:
```
cd ~/PATH_TO/catkin_ws/src
git clone git@github.com:intel-ros/realsense.git

catkin_init_workspace
cd ..
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
```
and don't forget to source:
```
echo "source ~/PATH_TO/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Now to make sure you can do RGBD you need to run:
```
sudo apt install ros-kinetic-rgbd-launch
```
At this point you can run 
```
roslaunch realsense2_camera rs_camera.launch
```
To stream the camera RGB data or: 
```
roslaunch realsense2_camera rs_rgbd.launch
```
To stream the RGBD data.

To do the visualization, you need to manually call `rviz`:
```
rviz
```
In `rviz`, you need to first specify `Global Options/Fixed Frame` as the `camera_link`, then add `camera` topic to visualize the RGB image or `pointcloud2` topic to visualize the RGBD topic. In regards to RGBD visulization, I found its better to set the `style` to `points` and the size to be 3 pixels for the best look. The camera does not see depth from a close shot, so it's probably best to try to look at a scene from about 50 cms or so.




# NVIDIA Graphic Card Setup on Ubuntu

If you have a fresh machine with a fancy GPU, such as a 1080TI, here are the steps to set up a new graphics card. For reference, I used [this page](https://blog.nelsonliu.me/2017/04/29/installing-and-updating-gtx-1080-ti-cuda-drivers-on-ubuntu/)

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
The you want to go to the [Binary Driver How To for NVIDIA](https://help.ubuntu.com/community/BinaryDriverHowto/Nvidia), and check the latest driver number, as of the date of this guide it was 390, then do this:
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

