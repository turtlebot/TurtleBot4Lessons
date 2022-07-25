### Prerequisites:
### Source installation
To manually install TurtleBot4 from source, clone the git repository:
```
cd ~/turtlebot4_ws/src
git clone https://github.com/turtlebot/turtlebot4_simulator.git
```
### Install dependencies:
```
cd ~/turtlebot4_ws
vcs import src < src/turtlebot4_simulator/dependencies.repos
rosdep install --from-path src -yi
```
### Build the packages:
```
source /opt/ros/galactic/setup.bash
colcon build --symlink-install
```
Now, paste your custom world inside below directory
```
cd ~/turtlebot4_ws/src/turtlebot4_simulator/turtlebot4_ignition_bringup/worlds
paste bug2_world.sdf
```
To launch the Tb4 onto custom world and to launch motion planner follow below launch commands:
```
ros2 launch ros2_tb4_visual_nav tb4_ignition.launch.py
ros2 launch ros2_tb4_visual_nav tb4_bug2.launch.py
```

Default goal location is set to X: 2 Y: 8