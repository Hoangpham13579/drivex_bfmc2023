# Self-driving car simulator

From Drivex_VGU team, an autonomous RC car simulation with scaled 1:10. This Github uses ROS 1 noetic for development.

To view full video demo, please visit [full demo](https://drive.google.com/file/d/12H4Xb3J2VLrR2vHwT21acXhTvu6A2s-H/view)

<img src="./images/demo_final_trim.gif" alt="drawing" width="" height=""/>

## Simulation features
- Autonomously drive on the designated map (highway, curves, intersection)
- Apply "Behavioral clonning", taking input from camera and output steering angle and speed 

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

To setup environment, please insert the bellow variables in your ~/.bashrc (note that you should modify the {YOUR_USER} to your local path)

```
source ~/catkin_ws/devel/setup.bash

# Simulation setup
export GAZEBO_MODEL_PATH="/home/{YOUR_USER}/catkin_ws/src/drivex_bfmc2023/src/models_pkg:$GAZEBO_MODEL_PATH"
export ROS_PACKAGE_PATH="/home/{YOUR_USER}/catkin_ws/src/drivex_bfmc2023/src:$ROS_PACKAGE_PATH"

# setup
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

To spawn the map & the car
```
roslaunch drivex_main map_with_cardrivex.launch
```

To make the car run
```
roslaunch drivex_main main_driving.launch model_name:=nvidia_speedSteerV1
```
Note that **nvidia_speedSteerV1** is the name of folder's model in package **drivex_driving**

To collect the dataset
```
roslaunch drivex_driving dataset_writing.launch
```

To train the Deep Learning model for behavioural cloning algorithm
```
python3 model_train.py --dataset_name {DATASET'S FOLDER NAME} --folder_name {MODEL'S FOLDER NAME}
```

To control the car manually from the keyboard (note that this command should run after spawning the map and the car)
```
rosrun drivex_showcar keyboard.py
```

To control the car with joy stick (the only option), please install dependencies and run the joy stick from [ROS joy stick installation](https://github.com/AutoMecUA/AutoMec-AD/wiki/Users'-guide-to-Software-installation)

## Reference
https://github.com/AutomecUA/AutoMec-AD

https://github.com/CIR-KIT/steer_drive_ros.git
