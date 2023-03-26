# camera_process
camera process with jetson nano

## equipment
- [realsense d435](https://www.intelrealsense.com/depth-camera-d435/)
- [Arducam IMX219](https://www.arducam.com/product/arducam-imx219-auto-focus-camera-module-drop-in-replacement-for-raspberry-pi-v2-and-nvidia-jetson-nano-camera/)

## software
- ubuntu 18.04 for jetson nano or jetson xviear nx
- ROS melodic


## install

### ubuntu on jetson
1. download sd card image from [jetson download center](https://developer.nvidia.com/embedded/downloads#?search=sd%20card%20image)
  - choose "Jetson Nano Developer Kit SD Card Image, JetPack 4.6" for nano, L4T 32.6.1, Ubuntu 18.04, Kernal 4.9, CUDA 10.2
  - choose "Jetson NX Developer Kit SD Card Image, JetPack 4.6" for Xavier NX, L4T 32.6.1, Ubuntu 18.04, Kernal 4.9, CUDA 10.2
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
2. [build from source](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md#building-from-source-using-native-backend)
  build librealsense2 and pyrealsense2 (-DBUILD_PYTHON_BINDINGS:bool=true) for python2.7 (-DPYTHON_EXECUTABLE=/usr/bin/python2.7)
  ```
  wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.50.0.zip
  unzip v2.50.0.zip
  cd librealsense-2.50.0



  ./scripts/patch-realsense-ubuntu-L4T.sh  

  sudo apt-get install git libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev -y
  ./scripts/setup_udev_rules.sh  
  mkdir build && cd build  

  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64
  export CUDA_HOME=/usr/local/cuda
  export PATH=$PATH:$CUDA_HOME/bin

  cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=release -DFORCE_RSUSB_BACKEND=true -DBUILD_WITH_CUDA=true -DPYTHON_EXECUTABLE=/usr/bin/python2.7 -DBUILD_PYTHON_BINDINGS:bool=true
  make -j2
  sudo make install

  echo "export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python2.7/pyrealsense2" >> ~/.bashrc
  source ~/.bashrc

  cd librealsense-2.50.0
  sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && udevadm trigger

  ```
  copy pyrealsense2 to /usr/local/lib/python2.7 if necessary

### realsense-ros, if need to use realsense2-camera
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


## ros interface
make a catkin_ws/src in jetson nano and download this repo
  ```
  git clone https://github.com/suneric/camera_process.git
  roslaunch camera_process ardu_camera.launch
  roslaunch camera_process realsense_camera.launch
  ```
note: sudo chmod +x all the launch file and script files

## verify ros in a remote computer
make a catkin_ws/src in your computer and download this repo
```
rosrun camera_process cam_ros_test.py --camera [ac|rs]
```

### ROS network
- ROS_MASTER_URI=http://ubuntu-Aurora-R7:11311
- ROS_HOSTNAME=ubuntu-jetson
- add hostname to /etc/hosts
```
127.0.1.1 ubuntu-jetson
192.168.1.7 ubuntu-Alienware-Aurora-R7
```

### setup service
create a service /etc/systemd/system/camera.service
```
[Unit]
Description="Camera Interface"
Wants=network-online.target
After=multi-user.target network.target network-online.target

[Service]
Type=simple
User=ubuntu
Group=ubuntu
ExecStart=/home/ubuntu/catkin_ws/src/camera_process/auto_start.sh
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

```
sudo chmod +x auto_start.sh
```

```
sudo systemctl enable camera.service
```
