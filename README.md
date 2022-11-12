# PExUAV
Open-Source Framework for Planetary Exploration UAV Autonomous Mission Planning


[![QUT Centre for Robotics Open Source](https://github.com/qcr/qcr.github.io/raw/master/misc/badge.svg)](https://qcr.github.io)
[![License: MIT](https://img.shields.io/badge/License-BSD-yellow.svg)](https://opensource.org/licenses/BSD-3-Clause)


## Contents

- [1) Synopsis](#1)
- [2) Getting the simulation environment going](#2)
- [3) Getting the emulation environment going](#3)
- [4) Tutorials](#4)
- [5) Common Issues and Solutions](#5)

<br>

<a id='1'></a>

## 1) Synopsis


The PExUAV framework provides:

- Installation instructions
- Code for Planetary Exploration UAV Autonomous Mission Planning. The framework relies on diverse software tools including Linux (Ubuntu 20.04), the Robotic Operation System (ROS1), C++, Python and Julia libraries.
- Source code that can be read for learning and teaching
- Python Notebooks for experiments results analysis.

<br>

<a id='2'></a>
## 2) Getting the simulation environment going

What you need before starting (a Linux machine):

-   You will need a machine with Ubuntu 20.04, installation instructions can be found [here](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview). (Raspberry Pi 4 is also supported, using [Ubuntu Mate 20.04](https://ubuntu-mate.org/raspberry-pi/). However, the emulator using Gazebo can be slow)

<!-- Python >= 3.6 -->

### 2.1) ROS Installation:
ROS Noetic is the current supported and tested version of ROS. To install ROS noetic, you can use the following terminal commands compilation (single commands separated by an empty line): 

```shell script
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl # if not already installed

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install ros-noetic-desktop -y
```
Also, we need some tools to help us manage ROS packages:

```shell script
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

sudo rosdep init

rosdep update
```

The commands presented previously are a compilation [from here](https://wiki.ros.org/noetic/Installation/Ubuntu). Ensure the previous commands successfully downloaded ROS noetic desktop and installed it. If ROS was installed with no errors proceed with the next steps, otherwise, please report the issues you founded [here](https://github.com/qutas/UAV4PE/issues).

### 2.2) Create ROS workspace to add packages

```shell script
mkdir -p ~/catkin_ws/src

cd ~/catkin_ws/

source /opt/ros/noetic/setup.bash

catkin_make
```


### 2.3) Install QUTAS Flight Stack

Let's install the Queensland University of Technology UAS flight stack using the following commands:

```shell script
wstool init src

cd ~/catkin_ws/

curl https://raw.githubusercontent.com/qutas/info/master/Stack/qfs_noetic.rosinstall > /tmp/qfs_noetic.rosinstall

wstool merge -t src /tmp/qfs_noetic.rosinstall

wstool update -t src

rosdep install --from-paths src --ignore-src -r -y

catkin_make

source ~/catkin_ws/devel/setup.bash
```
Then we need to install some packages to communicate with the UAV and the motion tracking system:

```shell script
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras -y

roscat mavros install_geographiclib_datasets.sh | sudo bash

mkdir -p ~/catkin_ws/launch

roscp mavros px4_pluginlists.yaml ./

roscp mavros px4_config.yaml ./

sudo apt-get install ros-noetic-vrpn-client-ros
```

### 2.4) Terminal setup

To set up the terminal to work with ROS, we need to include the ROS environment paths and other bits in the .bashrc file.

The .bashrc file is a special Linux file that runs commands at the start of every terminal. So for example, the ROS paths are loaded and available in every terminal you open.

The .bashrc file is normally located in your home directory and the '.' at the front of the file indicates that is a hidden file (so you normally do not see it when in the 'home' folder, at least you activate the visualization of hidden files).

To edit the .bashrc file you can do it using the file browser (after activating the hidden files visualization, pressing Ctrl+h) or with a terminal editor such as 'nano' with the following command:

```
nano ~/.bashrc
```
You should see a bunch of code inside the .bashrc file, (if not, there should be a path or name issue). Scroll down in the .bashrc, and add the following chunk of code at the end (you just need to do this one time, at least you format a new ubuntu, or do something else):

```
# ---------- UAV4PE help scripts ----------
# This ones ensure ROS works in every new terminal
source /opt/ros/noetic/setup.bash
# This ones load your workspace in every new terminal
source ~/catkin_ws/devel/setup.bash
 
# This function let you easy work with multiple ROS computers in the same network in every new terminal
disros() {
  # Setup for distributed ROS
  export ROS_IP="$(hostname -I | cut -d' ' -f1)"
  echo "Identifying as: $ROS_IP"
 
  if [ "$#" -eq 1 ]
  then
    export ROS_MASTER_URI="http://$1:11311"
    echo "Connecting to: $ROS_MASTER_URI"
  fi
}
# Start by default with as ROS master
disros
```

<!-- Check the tutorial section to sew how to launch it. -->

### 2.5) Install uav4pe_environments in ROS workspace src folder from GitHub

To install the last version of the uav4pe_environments repository we can use:
<!-- (ignore lines with #, those are bash comments): -->

```shell script
cd ~/catkin_ws/src

git clone https://github.com/qutas/uav4pe_environments

cd ~/catkin_ws

catkin_make

source ~/catkin_ws/devel/setup.bash
```
Well done! Your system is ready to launch the simulation environment. 

### 2.6) Launch simulation:
```
roslaunch uav4pe_environments load_map-16A.launch runSimulation:=True
```

## 3) Getting the simulation environment going

Emulation Installation steps (Tested on clean Ubuntu 20.04)

### 3.1) Install Gazebo Libraries
```
sudo apt-get install ros-noetic-gazebo-plugins

sudo apt-get install ros-noetic-gazebo-ros

sudo apt-get install ros-noetic-image-proc
```

### 3.2) Install PX4

```shell script
cd ~/

git clone https://github.com/PX4/PX4-Autopilot.git --recursive

bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

cd ~/PX4-Autopilot

make px4_sitl gazebo
```

If the installation is successful you should see a drone in Gazebo. You can close that window. More details can be found [here](https://docs.px4.io/main/en/simulation/gazebo.html).

### 3.3) Set up paths for Gazebo and models.

Scroll down in the .bashrc, and add to the end the following chunk of code (you just need to do this one time, at least you format a new ubuntu, or do something else):

```shell script
# ---------- UAV4PE  Emulation Environment Setup
# PX4 flight controller repository location
export PX4_DIR="$HOME/PX4-Autopilot"
 
## Gazebo environment variables
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/uav4pe_environments/models
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:$HOME/catkin_ws/src/uav4pe_environments/worlds
export LB_MODEL_PATH=${LB_MODEL_PATH}:$HOME/catkin_ws/devel/lib
 
## PX4 environment variables
source $PX4_DIR/Tools/setup_gazebo.bash $PX4_DIR $PX4_DIR/build/px4_sitl_default >~/.source_px4.log
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$PX4_DIR:$PX4_DIR/Tools/sitl_gazebo
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$PX4_DIR/build/px4_sitl_default/build_gazebo
```

<!-- ### 3.4) Automation Scripts

To use the simulation we need to launch multiple terminals. An easy way to do it is to add some automation scripts to the .bashrc file. Before adding the automation function we need first to install the Tmux tool:

```shell script
sudo apt install tmux
```


Scroll down in the .bashrc, and add to the end the following chunk of code (you just need to do this one time, at least you format a new ubuntu, or do something else):

```shell script
# ---------- UAV4PE load automation bash function using Tmux
function load_uav4pe_emulation (){
    echo "Starting Tmux session for uav4pe emulation environment"
    sleep 2
    tmux new-session -s uav4pe\; set -g mouse on; send-keys "roscore" C-m\; split-window -h -p 85\; select-pane -t 0\; split-window -v\;                   \
        send-keys "~/Desktop/./QGroundControl.AppImage" \; select-pane -t 2\; split-window -h\; select-pane -t 2\;                   \
        send-keys "sleep 5; roslaunch uav4pe_environments load_uav4pe.launch" C-m\; split-window -v -p 75\;               \
        send-keys "sleep 10; " \; split-window -v -p 60\;                                 \
        send-keys "sleep 5; roslaunch qutas_lab_450 environment.launch" C-m\; split-window -v \;                                        \
        send-keys "sleep 10; " \;  select-pane -t 6\;                                   \
        send-keys "rosrun uav4pe_navigation load_navigation" \;  split-window -v -p 85\;                      \
        send-keys "sleep 10; rosrun uav4pe_navigation send_input" C-m\; split-window -v -p 80\; select-pane -t 7\; split-window -h\; \
        send-keys "sleep 10; rostopic echo /uav4pe_navigation/command" C-m\; select-pane -t 9\;                                                 \
        send-keys "roslaunch uav4pe_mission_planner load_planner.launch runSimulation:=False" \; split-window -v -p 10\;                             \
        send-keys "tmux kill-session" \;
 
}
export -f load_uav4pe
``` -->


<br>

<a id='3'></a>

## 4) Tutorials (Coming soon...)

### 4.1) Run a single experiment:

### 4.2) Run multiple experiments:

### 4.3) Process data from experiments:

### 4.4) Plot and analyze data from experiments:

<br>

<a id='4'></a>

<p align="center">
	<img src=".gif">
</p>

<a id='8'></a>

## Common Issues and Solutions

See the common issues with fixes [here](https://github.com/qutas/UAV4PE/issues).