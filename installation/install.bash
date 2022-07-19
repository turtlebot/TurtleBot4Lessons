#!/bin/bash

sudo apt update && sudo apt install locales
echo ::::::::::::::::DONE installing locales!::::::::::::::::

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

echo :::::::::::::::Exporting UTF-8::::::::::::::::
export LANG=en_US.UTF-8
echo :::::::::::::::DONE!::::::::::::::::::::::::::

echo :::::::::::::::verify locale::::::::::::::::
locale  # verify settings
echo :::::::::::::::DONE!::::::::::::::::

echo :::::::::::::Verify Ubuntu Universe repository is enabled :::::::::::::::::
apt-cache policy | grep universe
echo ::::::::::::::DONE!:::::::::::::::::

echo If you don’t see an output line like the one above, then enable the Universe repository with these instructions.

sudo apt install software-properties-common
sudo add-apt-repository universe

echo :::::::::::::Adding the ROS 2 apt repository to your machine::::::::::::

sudo apt update
yes Y | sudo apt install curl
yes Y | sudo apt install gnupg 
yes Y | sudo apt install lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo :::::::::::::::::::Adding ROS2 Repo to source list::::::::::::::::::::::
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
echo :::::::::::::::::::DONE!:::::::::::::::::::::::::::::::::::::::::::::::::

sudo apt update
yes Y | sudo apt upgrade
echo :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::Installing ROS2 Galactic Desktop:::::::::::::::::::::::::
echo :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

yes Y | sudo apt install ros-galactic-desktop

echo ::::::::::::::::Sourcing ROS2 Galactic Desktop::::::::::::::::::::::::
source /opt/ros/galactic/setup.bash
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::Installation successfull::::::::::::::::::::::::::::::
echo ::::::::::::::::Installation successfull::::::::::::::::::::::::::::::
echo ::::::::::::::::Installation successfull::::::::::::::::::::::::::::::
echo ::::::::::::::::Installation successfull::::::::::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::Verifying ROS2 Distro:::::::::::::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
printenv ROS_DISTRO
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::Installing Required Packages::::::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

yes Y | sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget


yes Y |sudo apt install -y \
python3-colcon-common-extensions \
python3-rosdep \
python3-vcstool

echo ::::::::::::::::::::::::::::::::DONE!::::::::::::::::::::::::::::::::::::::

echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::Installing TurtleBot4 Packages::::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

yes Y | sudo apt update
yes Y | sudo apt install ros-galactic-turtlebot4-description \
ros-galactic-turtlebot4-msgs \
ros-galactic-turtlebot4-navigation \
ros-galactic-turtlebot4-node

sudo apt update
sudo apt install ros-galactic-turtlebot4-desktop

echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::Installing Ignition Gazebo::::::::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

yes Y | sudo apt-get update && sudo apt-get install wget
yes Y | sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
yes Y | wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
yes Y | sudo apt-get update && sudo apt-get install ignition-edifice

echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::Installing TurtleBot4 Simulator::::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
yes Y | sudo apt update
yes Y | sudo apt install ros-galactic-turtlebot4-simulator ros-galactic-irobot-create-nodes

echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::Instalation Successful::::::::::::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
