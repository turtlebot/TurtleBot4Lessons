#!/bin/bash

sudo apt update && sudo apt install locales
echo ::::::::::::::::Done installing locales!::::::::::::::::

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

echo :::::::::::::::Exporting UTF-8::::::::::::::::
export LANG=en_US.UTF-8
echo :::::::::::::::Done!::::::::::::::::

echo :::::::::::::::verify locale::::::::::::::::
locale  # verify settings
echo :::::::::::::::Done!::::::::::::::::

echo :::::::::::::Verify Ubuntu Universe repository is enabled :::::::::::::::::
apt-cache policy | grep universe
echo ::::::::::::::Done!:::::::::::::::::

echo If you don’t see an output line like the one above, then enable the Universe repository with these instructions.

sudo apt install software-properties-common
sudo add-apt-repository universe

echo :::::::::::::Adding the ROS 2 apt repository to your machine::::::::::::

sudo apt update
yes Y | sudo apt install curl
yes Y | sudo apt install gnupg 
yes Y | sudo apt install lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo :::::::::::::::::::Adding ROS2 Repo to source list:::::::::::::::::::
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
echo :::::::::::::::::::Done!:::::::::::::::::::

sudo apt update
yes Y | sudo apt upgrade
echo :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::Installing ROS2 Galactic Desktop:::::::::::::::
echo :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

yes Y | sudo apt install ros-galactic-desktop

echo ::::::::::::::::Sourcing ROS2 Galactic Desktop:::::::::::::::
source /opt/ros/galactic/setup.bash
echo ::::::::::::::::Installation successfull::::::::::::::::::::
echo ::::::::::::::::Installation successfull::::::::::::::::::::
echo ::::::::::::::::Installation successfull::::::::::::::::::::
echo ::::::::::::::::Installation successfull::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::Verifying ROS2 Distro:::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
echo ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
printenv ROS_DISTRO
