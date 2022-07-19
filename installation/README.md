### Instructions to Install Ubuntu 20.04 on Virtual Machine

[Refer this link to install ubuntu on virtualbox or VMware](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview)

### Instructions to Install ROS2 and TurtleBot 4 packages on a **single click**

In your terminal
```
git clone https://github.com/turtlebot/TurtleBot4Lessons.git
cd TurtleBot4Lessons/installation
bash install.bash
enter your administrator passkey to run and install scripts (** sudo access required)
```

Now sit back and relax for a while untill below packages gets installed
- ROS2 galactic
- Turtlebot4 packages 
- other required packages like 
  - build-essential
  - cmake
  - git 
  - python3-colcon-common-extensions
  - python3-flake8
  - python3-pip
  - python3-pytest-cov
  - python3-rosdep
  - python3-setuptools
  - python3-vcstool
  - wget
  - python3-colcon-common-extensions
  - python3-rosdep
  - python3-vcstool

Note** 
- You might be asked to enter "Y" while installing packages in some rear instances. During such situations, please enter Y to process installation without hindrances
- This installation process is valid on Virtual machine as well as on Ubuntu Desktop version
