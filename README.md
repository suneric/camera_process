# camera_process
camera process with jetson nano

## equipment
- [realsense d435](https://www.intelrealsense.com/depth-camera-d435/)
- [Arducam IMX219](https://www.arducam.com/product/arducam-imx219-auto-focus-camera-module-drop-in-replacement-for-raspberry-pi-v2-and-nvidia-jetson-nano-camera/)

## software
- ubuntu 18.04 for jetson nano
- ROS melodic


## install

### ubuntu on jetson nano
1. download sd card image from [jetson download center](https://developer.nvidia.com/embedded/downloads#?search=sd%20card%20image)
  - choose "Jetson Nano Developer Kit SD Card Image, 4.6.1"
2. [flash the image](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#write) to you sd card with Etcher
3. setup and first boot your system

### ROS melodic
- [install ROS melodic on ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu)

### realsense
1. [sdk installation](https://dev.intelrealsense.com/docs/nvidia-jetson-tx2-installation)
  ```
  # register key
  sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

  # add server for ubuntu 18.04
  sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main" -u

  # install sdk
  sudo apt-get install librealsense2-utils
  sudo apt-get install librealsense2-dev

  # varify the installation
  realsense-viewer  
  ```
2. realsense-ros
```
sudo apt install ros-melodic-realsense2-camera
```


### arducam
1. connect and verify
  ```
  git clone https://github.com/ArduCAM/Nvidia_jetson.git
  cd IMX219_AutoFocus
  python AutoFocus.py
  ```
2.
