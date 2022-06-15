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

* pip / pip3 (depend on python version) -\
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
> [!WARNING] DON'T USE BOTH git INSTALLATIONS UNDER ANY CIRCUMSTANCES!
```bash
sudo apt install git-all
```

* ROS Wrapper for Intel® RealSense™ Devices - ([GitHub](https://github.com/IntelRealSense/realsense-ros))
```bash
sudo apt install ros-melodic-realsense2-camera
sudo apt install ros-melodic-realsense2-description
```

* imu_tools (ROS package which is includes imu madgwick filter)
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
git clone https://github.com/GuideMeDev/depth_clustering_realsense.git (recommended)
cd ..
catkin_make
```


### additional installations (recommended)
* make ROS workspace compatible for python2 alongside python3
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


## System User Manual:
### For Further Explanations - refer to guide_me_ros/Algo/Documentation/Migdal_Or_Real_Time.docx 
[First Nuc password : qq , Second Nuc Password : q]
[Anydesk password: 123456L!]
[To switch the NUC between remote screen and physical screen mode - "rem_screen.sh" and "phys_screen.sh"]
1. Navigate to guide_me_ros -> scripts -> bash automation, and follow the steps below:
2. [for remote users] Activate floating keyboard with the script "keyb.sh"
3. Activate the realsense camera by pressing on "cam_launch.sh".
4. Activate the vibration usb device with "vibration_usb.sh".
5. launch the algorithm's script with "algo.sh"

[In all of these steps, the user password is required upon activation]

#### Some documentation videos to help you understand how to run the examples

* MATLAB

[screenshot video](https://drive.google.com/file/d/1rhiD_EtPqLLo9cVh6T4AxgHytVJ7IKfW/view?usp=sharing)

[webcam video](https://drive.google.com/file/d/1Wdv-fHuqVAz7jxolYuE3gGCZEKZuvgsw/view?usp=sharing)

* python

[screenshot video](https://drive.google.com/file/d/1FYwYNqP-A05kN1gFn2AOkY7XnlN1-QOK/view?usp=sharing)

[webcam video](https://drive.google.com/file/d/1NHHETOlIXJpJY5XEiI88ae1Q-zH_ZJVw/view?usp=sharing)


#### Documentation videos

[screenshot video](https://drive.google.com/file/d/1Yzw4zYNAInqfZi3sKzdEYf4n6wITRwMs/view?usp=sharing)

[webcam video](https://drive.google.com/file/d/1WVBhSKD-wPd2ZxDWMlfQ9U6kBPCHd2yJ/view?usp=sharing)


## ROS bag data

[2021-09-29-15-58-47_mat](https://drive.google.com/drive/folders/16rqd9QOSTA8Iqoz_WU-HwOWH4pjT6iHJ?usp=sharing)

## 1. SLAM and control algorithm with Data, 2. Three video demos, 3. Semantics demo

(https://drive.google.com/drive/folders/1PUcnlzjTR7WZk8YROn7DSYsnxtSALBJS?usp=sharing)

## Set Camera ROI, setpoint and exposure time Using Dynamic Reconfigure Params and realsense-viewer

### ROS Dynamic Reconfigure Params

1. run realsense driver
```bash
roslaunch guide_me_ros rs_camera.launch
```
2. Thene use the following command

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

* ROI and exposure

![Dynamic Reconfigurer ROI exposure](https://github.com/GuideMeDev/guide_me_ros/blob/master/doc/rqt_screenshot.png)

[for more info go here](https://github.com/IntelRealSense/realsense-ros#set-camera-controls-using-dynamic-reconfigure-params)

### realsense-viewer

open realsense-viewer

```bash
realsense-viewer
```

* ROI and exposure

![realsense-viewer ROI exposure](https://github.com/GuideMeDev/guide_me_ros/blob/master/doc/realsense-viewer_screenshot.png)

* setpoint

![realsense-viewer setpoint](https://github.com/GuideMeDev/guide_me_ros/blob/master/doc/realsense-viewer_2_screenshot.png)


[for more info go here](https://github.com/GuideMeDev/guide_me_ros-private/blob/master/doc/BKMs_For_Tuning_Intel_RealSense_D4xx_Cameras_Whitepaper_2.0.pdf)
