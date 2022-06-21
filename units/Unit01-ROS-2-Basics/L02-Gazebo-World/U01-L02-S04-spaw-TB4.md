template: ../media/TB4Template.pptx

# Objective: Unit 1 Lesson 2 Section 4

## Spawn a Turtle Bot 4 on to Gazebo world

1. To launch turtle bot 4 with warehouse world, below follow below steps. The turtlebot4_ignition_bringup package contains launch files and configurations to launch Ignition Gazebo.

```
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ignition-edifice ros-galactic-turtlebot4-simulator ros-galactic-irobot-create-nodes
```
(or)

- Create a new workspace with a src folder in it and colcon build the workspace

```
mkdir turtle_ws
cd turtle_ws && mkdir src
colcon build
```
- clone the Turtlebot4 Simulator

```
cd turtle_ws/src
git clone https://github.com/turtlebot/turtlebot4_simulator.git
cd .. && colcon build
```
- Source the package

```
. install/setup.bash
```
- Launch Turtle Bot 4

```
ros2 launch turtlebot4_ignition_bringup ignition.launch.py
```
