# Description of the Project:

This project educates to implement a Bug2 Motion planner Algorithm using Extetented Kalman filter state estimation.
Algorithm in a nutshell:
- Initially, the TB4 heads towards goal on the sg-line (A line between start and goal points)
- If an obstacle/wall is on the way, follow it until you encounter the sg-line
closer to the goal.
- Consider the point where turtlebot4 reaches the sg-line first as
reach point. Similarly, consider the point where turtlebot4 leaves the
sg-lines as leave point
- TB4 leaves the obstacle/wall and continue toward the goal

![179041882-b5b8f156-bbd7-460d-b84a-1b2123f99494](https://user-images.githubusercontent.com/24978535/181306870-fc7721d5-ca14-4c5b-be4c-2bc78e6cc70d.png)

# Instructions to run the Project:

### Create a ROS2 Workspace
```
cd
mkdir workspace
```
### Create a sub-directory with src and colcon build your workspace
```
cd workspace
mkdir src
colcon build
```
### Clone the Turtlebot4Lessons Repository into src folder
```
cd workspace/src/
git clone https://github.com/turtlebot/TurtleBot4Lessons.git
```

### View your Source files
```
cd Turtlebot4Lessons
```

# Instructions to setup Turtlebot4 Simulation source installation and Motion Planner project

### TurtleBot4 Simulation Source installation
To manually install TurtleBot4 simulation from source, clone the git repository:
```
cd ~/workspace/src
git clone https://github.com/turtlebot/turtlebot4_simulator.git
```
### Install dependencies:
```
cd ~/workspace
vcs import src < src/turtlebot4_simulator/dependencies.repos
rosdep install --from-path src -yi
```
### Build the TurtleBot4 simulation package:
```
. install/setup.bash
colcon build --symlink-install
```
### Now, download your custom world inside below directory
```
cd ~/workspace/src/turtlebot4_simulator/turtlebot4_ignition_bringup/worlds
wget https://raw.githubusercontent.com/sumedhreddy90/TurtleBot4Lessons/ROS2-visual-Nav/projects/unit02_simulation_motion_planner/worlds/bug2_world.sdf
```
### Edit 'world' DeclareLaunchArgument in ignition.launch.py

```
cd ~/workspace/src/turtlebot4_simulator/turtlebot4_ignition_bringup/launch/
gedit ignition.launch.py
goto line 65 and change default_value='bug2_world'
```

### Launch Turtlebot4 in Ignition Gazebo

To launch the TB4 onto custom world run below launch command:
```
ros2 launch unit02_simulation_motion_planner tb4_ignition.launch.py
```
### To undock the Turtlebot4 from the Charging Dock

Execute below launch command in your terminal for undocking Turtlebot4
```
ros2 launch unit02_simulation_motion_planner tb4_undock.launch.py 
```

### To launch motion planner alogirthm to given goal position

```
ros2 launch unit02_simulation_motion_planner tb4_bug2.launch.py
```

Default goal location is set to X: 8.0 Y: 10.0

### To pass goal postion (TODO from command line)

```
cd ~/workspace/src/TurtleBot4Lessons/projects/unit02_simulation_motion_planner/unit02_simulation_motion_planner/
gedit tb4_publish_goal.py
goto line 37 and declare your desired goal_pose
self.declare_parameter('goal_pose', [8.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0])
cd ~/workspace
colcon build
```
