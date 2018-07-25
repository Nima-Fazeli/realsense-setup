
# Step-by-Step guide to the manual library installations

The libraries we're going to need:

1. OpenCV
2. PCL (Point Cloud Library)
3. RealSense 2.0 SDK

I have created shell scripts to install each of these libraries. You can just run these scripts to automate the process. Shell scripts (in preferred order): ([OpenCV](https://github.com/Nima-Fazeli/realsense-setup/tree/master/bash_install_opencv), [PCL](https://github.com/Nima-Fazeli/realsense-setup/tree/master/bash_install_pcl), [RealSense SDK](https://github.com/Nima-Fazeli/realsense-setup/tree/master/bash_install_rs_sdk))


Here are the steps for each library if you want to do it manually:

## OpenCV

This is a super useful library to work with images, there are a bunch of functionalities we'll use from this package. Just a word of caution, the latest version of OpenCV does not play too nicely and needs a little care, but we'll take care of that in the linking phase.

To install OpenCV you can refer to [OpenCV Installation](https://docs.opencv.org/3.4/d7/d9f/tutorial_linux_install.html).

The TLDR version of it is:
```
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
```
Optionally:
```
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
```

Next clone the OpenCV git to a directory you don't really plan on visiting much:
```
mkdir ~/<working_dir>
git clone https://github.com/opencv/opencv.git
```
Once done, navigate to the directory:
```
cd ~/opencv
mkdir build
cd build
```
Then run:
```
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
```
You could alternatively use `cmake-gui`, if you want to, follow the instructions on the OpenCV installation page.
Now you need to `make` in the build directory (the j argument selects the number of cores):
```
make -j7
```
Finally we install the libraries:
```
sudo make install
```
At this point you have the minimal necessary OpenCV libraries to work with.

**IMPORTANT- CMAKE Additions**: You'll need these in your `CMakeLists.txt`:
```
find_package(Boost REQUIRED)
find_package(OpenCV 3 REQUIRED
  COMPONENTS
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
  CONFIG
)
```

## PCL - Point Cloud Library

PCL depends on VTK, so first install that:
```
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
```

The preferred method of installing the PCL libraries is through the binaries:
```
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all
```
You can also install from source following [PCL from Source](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_posix.php).

## RealSense 2.0 SDK

These instruction follow on from [this tutorial](https://github.com/Nima-Fazeli/realsense-setup), but a little more optimized for this purpose. Just make sure you're on **Kernel 4.13**!! 

First you need to navigate to [RealSense SDK Github](https://github.com/IntelRealSense/librealsense). Here you'll find instructions for the installation.

If you're on Ubuntu, which we are going to assume you are, you can instead go to [Ubuntu Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md), and follow instructions.

TLDR:
```
echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | sudo tee /etc/apt/sources.list.d/realsense-public.list
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE
```
Wait for the okay for the key, or repeat the last instruction until you get it. Then make sure to **unplug** any cameras and:
```
sudo apt-get update
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```
You can test this installation by running `realsense-viewer`.

**IMPORTANT-Where are the Libraries**: This actually matters for linking and cpp compilations. Navigate to:
```
/usr/local/lib/cmake
```
This path will be put into a `CMakeLists.txt` later on. For some reason I could not find the realsense2 libraries without explicitly telling CMake where to look:
```
set( realsense2_DIR "/usr/local/lib/cmake" CACHE PATH "Path to librealsense2 config directory." )
find_package( realsense2 REQUIRED )
```
This SDK requires `c++11`, so we do need to later on add:
```
add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 11)
```
To the top of our `CMakeLists.txt` as well, it's easy to miss this point.
