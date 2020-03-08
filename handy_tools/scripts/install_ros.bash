#! /bin/bash

NC='\033[0m' # No Color
RED='\033[0;30m'
GREEN='\033[0;32m'
WHITE='\033[1;37m'

if [ -z $1 ]
then
 ROS_VERSION='melodic'
else
 ROS_VERSION=$1
fi

echo -e "${WHITE}[ROS - Installation]${NC}"

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install -y ros-${ROS_VERSION}-desktop-full
sudo rosdep init
rosdep update

echo "source /opt/ros/${ROS_VERSION}/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install -y python-rosinstall
sudo apt install -y python-rosinstall-generator
sudo apt install -y python-wstool
sudo apt install -y build-essential
sudo apt install -y python-catkin-tools

dpkg -s ros-${ROS_VERSION}-desktop-full &> /dev/null
if [ $? -eq 0 ]; then
	echo -e "${GREEN}ROS is installed!${NC}"
	echo -e "${WHITE}[ROS - Workspace Configuration]${NC}"
	source /opt/ros/${ROS_VERSION}/setup.bash

	mkdir -p ~/catkin_ws/src
	(cd ~/catkin_ws/ && catkin_make)
	source ~/catkin_ws/devel/setup.bash
	echo "source /home/grvc/catkin_ws/devel/setup.bash" >> ~/.bashrc
else
	echo -e "${RED}[ERROR]Instalation failed, please try again${NC}"
fi
