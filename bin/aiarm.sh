#!/bin/bash

sudo chmod 666 /dev/ttyXCar
sudo chmod 666 /dev/ttyUSB*
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roslaunch aiarm aiarm-controller.launch &
sleep 15
cd /home/zonesion/catkin_ws/src/aiarm/script
../vnode/xarm/release/vnode-xarm &
python3 main.py
sleep 99999
