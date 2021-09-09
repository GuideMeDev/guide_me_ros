# guide_me_ros
Some ROS files and examples for the project.


## Dependencies
* Ubunt 18 LTS - (you may use the instructions from [here](https://www.itzgeek.com/how-tos/linux/ubuntu-how-tos/how-to-install-ubuntu-18-04-alongside-with-windows-10-or-8-in-dual-boot.html))
* ROS melodic - ([installation](http://wiki.ros.org/melodic/Installation/Ubuntu))
* python2.7 or above recommended python3.* - (no installation needed)
* MATLAB 2018 + ROS Toolbox


## installation
* First of all update installations source
```bash
sudo apt update
```

* pip / pip3 (depending on python version) -\
pip (python2.*) use the command:
```bash
sudo apt install python-pip
```
pip3 (python3.*) use the command:
```bash
sudo apt install python3-pip
```
you may install both if needed

* git -\
installation:
```bash
sudo apt install git
```
sometimes you will need this command instead
> [!WARNING] DON'T USE BOTH INSTALLATIONS UNDER ANY CIRCUMSTANCES!
```bash
sudo apt install git-all
```

* ROS Wrapper for Intel® RealSense™ Devices - ([GitHub](https://github.com/IntelRealSense/realsense-ros))
```bash
sudo apt install ros-melodic-realsense2-camera
sudo apt install ros-melodic-realsense2-description
```

* imu_tools (ROS package which is including imu_filter_madgwick)
```bash
sudo apt install ros-melodic-imu-tools
```

* ROS Environment
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
echo "source $HOME/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

* package installation
```bash
cd ~/catkin_ws/src
git clone https://github.com/GuideMeDev/guide_me_ros.git
git clone https://github.com/GuideMeDev/depth_clustering_releases.git (recommended)
cd ..
catkin_make
```


### additional installations (recommended)
* ROS python2 alongside python3
```bash
cd ~/catkin_ws/src/guide_me_ros/bash/
sudo chmod +x ros-workspace_py2-alongside-py3.sh
./ros-workspace_py2-alongside-py3.sh
```

* ROS cv_bridge Python3 ([for more information](https://cyaninfinite.com/ros-cv-bridge-with-python-3/))\
installation instractions:
1. Dependencies
```bash
sudo apt install python-catkin-tools python3-dev python3-numpy
sudo apt install python-catkin-pkg  python-rospkg
```
2. Workspace
```bash
cd
mkdir -p ~/cvbridge_build_ws/src
cd ~/cvbridge_build_ws/src
git clone -b noetic https://github.com/ros-perception/vision_opencv.git
sed -i 's/find_package(Boost REQUIRED python37)/find_package(Boost REQUIRED python3)/' vision_opencv/cv_bridge/CMakeLists.txt
cd ~/cvbridge_build_ws
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin config --install
catkin build cv_bridge
echo "source $HOME/cvbridge_build_ws/install/setup.bash --extend" >> ~/.bashrc
```
* Nvidia driver + cuda + cudnn (Nvidia GPU is required)
```bash
cd ~/catkin_ws/src/guide_me_ros/bash/
sudo chmod +x nvidia-450_cuda-11.0_cudnn-8.0.5_setup.sh
./nvidia-450_cuda-11.0_cudnn-8.0.5_setup.sh
```

## Runing
* subscribe to rostopic with matlab (example) - subscribe to /imu/data and /camera/depth/color/points topics, prints the orientation and draws pointcloud2
1. run realsense driver and madgwick filter
```bash
roslaunch guide_me_ros rs-camera_imu_filter_madgwick.launch
```
2. Then open matlab, bay runing the folowing commands in new terminal
```bash
cd ~/catkin_build/src/guide_me_ros/mat
matlab .
```
In the new matlab window, open the file subscriber_example.m and run it (F5)
