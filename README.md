<img src="https://github.com/Hoangpham13579/drivex_bfmc2023/blob/7047ca1919d652c1c4906135ad6ece62a107b6f1/drivex_sim.png" width=80% height=80%>

# Drivex_VGU team BFMC 2023

Drivex_VGU, an autonomous RC car with scaled 1:10 for Bosch Future Mobility Challenge (BFMC)

Link for OS and ROS installation [wiki](https://github.com/AutoMecUA/AutoMec-AD/wiki/Users'-guide-to-Software-installation) from [AutoMec-AD](https://github.com/AutomecUA/AutoMec-AD) dependencies github installation

## Quick step-by-step
To install odometry packages
```
sudo apt-get install ros-noetic-ros-controllers ros-noetic-ackermann-msgs ros-noetic-navigation
```

And please clone [this repository](https://github.com/CIR-KIT/steer_drive_ros) to your ROS workspace (default is ~/catkin_ws/src). Change the git branch to the melodic-devel branch. This process can be done with the following commands:
```
cd ~/catkin_ws/src
git clone https://github.com/manuelgitgomes/steer_drive_ros.git
cd steer_drive_ros
git switch melodic-devel

# Move to default ~/catkin_ws workspace & run
catkin_make
```

To setup environment, please insert in your .bashrc/

```
source ~/catkin_ws/devel/setup.bash

# BFMC Simulation setup
export GAZEBO_MODEL_PATH="/home/{YOUR_USER}/catkin_ws/src/drivex_bfmc2023/src/models_pkg:$GAZEBO_MODEL_PATH"
export ROS_PACKAGE_PATH="/home/{YOUR_USER}/catkin_ws/src/drivex_bfmc2023/src:$ROS_PACKAGE_PATH"

# BFMC setup
export GAZEBO_MODEL_PATH="`rospack find drivex_driving`/models:${GAZEBO_MODEL_PATH}"
export DRIVEX_DRIVING="`rospack find drivex_driving`"
export GAZEBO_MODEL_PATH="`rospack find drivex_driving`:$GAZEBO_MODEL_PATH"

export ROS_PACKAGE_PATH="/home/{YOUR_USER}/catkin_ws/src/drivex_bfmc2023/src:$ROS_PACKAGE_PATH"
```

Then type
```
source ~/.bashrc

# Move to default ~/catkin_ws workspace & run
catkin_make
```

And please install the following dependencies:
```
pip3 install -r requirements.txt
```
Note that we should **install Pytorch GPU** to run the model for behavioural cloning algorithm. The installation guide of Pytorch GPU on Ubuntu can be found [here](https://www.youtube.com/watch?v=4LvgOmxugFU)

To run the BFMC map & spawn the car
```
roslaunch drivex_main map_with_cardrivex.launch
```

To make the car run
```
roslaunch drivex_main main_driving.launch model_name:=model_nvidia_bfmcV4
```
Note that **model_nvidia_bfmcV4** is the name of folder's model in package **drivex_driving**

To collect the dataset
```
roslaunch drivex_driving dataset_writing.launch
```

To train the Deep Learning model for behavioural cloning algorithm
```
python3 model_train.py --dataset_name {DATASET'S FOLDER NAME} --folder_name {MODEL'S FOLDER NAME}
```

To control the car with joy stick (the only option), please install dependencies from [ROS joy stick installation](https://github.com/AutoMecUA/AutoMec-AD/wiki/Users'-guide-to-Software-installation)

## Reference
https://github.com/AutomecUA/AutoMec-AD

https://github.com/CIR-KIT/steer_drive_ros.git
