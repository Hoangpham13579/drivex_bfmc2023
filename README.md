<img src="https://github.com/Hoangpham13579/drivex_bfmc2023/blob/7047ca1919d652c1c4906135ad6ece62a107b6f1/drivex_sim.png" width=80% height=80%>

# Drivex_VGU team BFMC 2023

Drivex_VGU, an autonomous RC car with scaled 1:10 for Bosch Future Mobility Challenge (BFMC)

## Quick step-by-step
To setup environment, please insert in your .bashrc/

```
export GAZEBO_MODEL_PATH="`rospack find drivex_driving`/models:${GAZEBO_MODEL_PATH}"
export DRIVEX_DRIVING="`rospack find drivex_driving`"

# BFMC simulator setup
export GAZEBO_MODEL_PATH="`rospack find drivex_driving`:$GAZEBO_MODEL_PATH"
export ROS_PACKAGE_PATH="*Your path*/drivex_bfmc2023/src:$ROS_PACKAGE_PATH"
```
Then type ```source ~/.bashrc```

And please install the following dependencies:

```
sudo apt-get install ros-noetic-ros-controllers ros-noetic-ackermann-msgs ros-noetic-navigation
```
```
pip3 install -r requirements.txt
```
Note that we should **install Pytorch GPU** to run the model for behavioural cloning algorithm (required!)

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
python3 model_train.py --dataset_name *dataset's folder name* --folder_name *model's folder name*
```

## Reference
https://github.com/AutomecUA/AutoMec-AD

https://github.com/CIR-KIT/steer_drive_ros.git
