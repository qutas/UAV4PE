# PExUAV
Open-Source Framework for Planetary Exploration UAV Autonomous Mission Planning


[![QUT Centre for Robotics Open Source](https://github.com/qcr/qcr.github.io/raw/master/misc/badge.svg)](https://qcr.github.io)
[![License: MIT](https://img.shields.io/badge/License-BSD-yellow.svg)](https://opensource.org/licenses/BSD-3-Clause)


## Contents

- [Synopsis](#1)
- [Getting going](#2)
- [Tutorials](#3)
- [Common Issues and Solutions](#4)

<br>

<a id='1'></a>

## Synopsis


The PExUAV framework provides:

- Installation instructions
- Code for Planetary Exploration UAV Autonomous Mission Planning. The framework relies on diverse software tools including Linux (Ubuntu 20.04), the Robotic Operation System (ROS1), C++, Python and Julia libraries.
- Source code that can be read for learning and teaching
- Python Notebooks for experiments results analysis.

<br>

<a id='2'></a>
## Getting going

What you need before starting (a Linux machine):

-   You will need a machine with Ubuntu 20.04, installation instructions can be found [here](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview). (Raspberry Pi 4 is also supported, using [Ubuntu Mate 20.04](https://ubuntu-mate.org/raspberry-pi/). However, the emulator using Gazebo can be slow)

<!-- Python >= 3.6 -->

### ROS Installation:
ROS Noetic is the current supported and tested version of ROS. To install ROS noetic, you can use the following terminal commands compilation (single commands separated by an empty line): 

```shell script
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl # if not already installed

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install ros-noetic-desktop -y

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

sudo rosdep init

rosdep update
```
The commands presented previously are a compilation [from here](https://wiki.ros.org/noetic/Installation/Ubuntu). The commands download ROS noetic desktop version and installed it. The commands also install needed python libraries for ROS.

### Create ROS workspace to add packages

```shell script
mkdir -p ~/catkin_ws/src

cd ~/catkin_ws/

source /opt/ros/noetic/setup.bash

catkin_make
```


### Install QUTUAS Flight Stack

The Queensland University of Technology UAS Flight Stack is installed using the following commands:

```shell script
wstool init src

cd ~/catkin_ws/

curl https://raw.githubusercontent.com/qutas/info/master/Stack/qfs_noetic.rosinstall > /tmp/qfs_noetic.rosinstall

wstool merge -t src /tmp/qfs_noetic.rosinstall

wstool update -t src

rosdep install --from-paths src --ignore-src -r -y

catkin_make

source ~/catkin_ws/devel/setup.bash

sudo apt install ros-noetic-mavros 
ros-noetic-mavros-extras -y

roscat mavros install_geographiclib_datasets.sh | sudo bash

mkdir -p ~/catkin_ws/launch

roscp qutas_lab_450 px4_flight_control.launch ~/catkin_ws/launch/control.launch

roscp mavros px4_pluginlists.yaml ./

roscp mavros px4_config.yaml ./

sudo apt-get install ros-noetic-vrpn-client-ros
```

### Install PX4


```shell script
cd ~/

git clone https://github.com/PX4/PX4-Autopilot.git --recursive

bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

cd ~/PX4-Autopilot

make px4_sitl gazebo
```


### From GitHub

To install the bleeding-edge version from GitHub

```shell script
git clone https://github.com/qutas/PExUAV
cd PExUAV
pip3 install -e .
```

<br>

<a id='3'></a>

## Tutorials

### 1) Run a single experiment:

### 2) Run multiple experiments:

### 2) Process data from experiments:

### 4) Plot and analyse data from experiments:

<br>

<a id='4'></a>

<p align="center">
	<img src=".gif">
</p>

<a id='8'></a>

## Common Issues and Solutions

See the common issues with fixes [here](https://github.com/qutas/PExUAV/issues).