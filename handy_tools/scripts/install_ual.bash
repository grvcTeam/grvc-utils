#! /bin/bash

NC='\033[0m' # No Color
YELLOW='\033[1;33m'
ROS_VERSION=$(rosversion -d)

echo -e "${YELLOW}[GRVC UAL]${NC}"
sudo apt install -y libeigen3-dev
sudo apt install -y ros-${ROS_VERSION}-mavros
sudo apt install -y ros-${ROS_VERSION}-mavros-extras
sudo apt install -y ros-${ROS_VERSION}-geodesy
sudo apt install -y ros-${ROS_VERSION}-joy

(cd ~/catkin_ws/src/ && git clone https://github.com/grvcTeam/grvc-ual.git)
(cd ~/catkin_ws/src/grvc-ual && ./configure.py)

sudo geographiclib-get-geoids egm96-5

sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager

echo -e "${YELLOW}[BUILD WORKSPACE]${NC}"
(cd ~/catkin_ws/ && catkin_make)




