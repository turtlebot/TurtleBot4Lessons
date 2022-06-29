template: ../media/TB4Template.pptx

# Unit 1 Lesson 2 Create Empty Gazebo World 

### Objective

* Understand Simulation with Ignition Gazebo and ROS 2
* How to create an empty Gazebo world.
* How to add a few objects on to Gazebo world.
* How to spawn a TurtleBot4 on to Gazebo world. 
* Understanding Launch files
* Creating your own Gazebo visual assests

### Create Empty Gazebo World

* For creating a empty world or custom world, please follow tutorials on [Gazebo_Docs](https://gazebosim.org/docs/citadel/sdf_worlds). 

* The tutorial will help you learn how to build a world using SDF, and how to add models to it. It also explains all the plugins and gui required for simulating a custom world.

![Empty Gazebo World](../media/empty_gazebo.png)

### Steps 

* As a quick guide, create an empty SDF file and copy paste below xml code to create an empty gazebo world.

    * Create an empty .sdf file

```
touch empty_world.sdf
```

    * Edit the empty_world.sdf file and copy below XML code or simply you can launch an exisiting pre-built SDF file

```
gedit empty_world.sdf
```
    
    * Run the SDF file

```
ign gazebo empty.sdf

```

### Explaining XML: Defining a world

* Create an SDF file with XML tag

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="empty_world">
  ...
  </world>
</sdf>
```
### Explaining XML: Add Plugins

* Add Plugins to your SDF as per your requirement

```xml
<plugin
    		filename="libignition-gazebo-physics-system.so"
   		name="ignition::gazebo::systems::Physics">
	</plugin>
	<plugin
    		filename="libignition-gazebo-user-commands-system.so"
    		name="ignition::gazebo::systems::UserCommands">
	</plugin>
	<plugin
    		filename="libignition-gazebo-scene-broadcaster-system.so"
    		name="ignition::gazebo::systems::SceneBroadcaster">
	</plugin>
```
### Explaining XML: Add GUI 

* Now let's define the GUI. Under the <gui> tag we specify anything related to the GUI of Ignition. [ignition-gui](https://github.com/gazebosim/gz-gui) has a bunch of plugins to choose from. We will add the ones that are necessary to get our world up and running with basic functionality.
```xml
<gui fullscreen="0">
  .
  .
  .
	
</gui>

```
### Explaining XML: Add GUI- scene 3D plugin 

* add scene 3D plugin to GUI. The GzScene3D plugin is responsible for displaying the 3D scene of our world.

```
    <!-- 3D scene -->
  <plugin filename="GzScene3D" name="3D View">
      <ignition-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
      </ignition-gui>

      <engine>ogre2</engine>
      <scene>scene</scene>
      <ambient_light>1.0 1.0 1.0</ambient_light>
      <background_color>0.8 0.8 0.8</background_color>
      <camera_pose>-6 0 6 0 0.5 0</camera_pose>
  </plugin>

```



###  Explaining XML: Add GUI- world control plugin (Contin.)

* The World control plugin is responsible for controlling the world.  Some of its properties are the following play_pause, stats_topic, start_paused

```
<plugin filename="WorldControl" name="World control">
    <ignition-gui>
        <title>World control</title>
        <property type="bool" key="showTitleBar">false</property>
        <property type="bool" key="resizable">false</property>
        <property type="double" key="height">72</property>
        <property type="double" key="width">121</property>
        <property type="double" key="z">1</property>

        <property type="string" key="state">floating</property>
        <anchors target="3D View">
        <line own="left" target="left"/>
        <line own="bottom" target="bottom"/>
        </anchors>
    </ignition-gui>

    <play_pause>true</play_pause>
    <step>true</step>
    <start_paused>true</start_paused>
    <service>/world/world_demo/control</service>
    <stats_topic>/world/world_demo/stats</stats_topic>
</plugin>
```
### Explaining XML: Add GUI- world stats plugin (Contin.)

* The World stats plugin is responsible for displaying the world statistics, <sim_time>, <real_time>, <real_time_factor> and <iterations> 

```xml
<!-- World statistics -->
<plugin filename="WorldStats" name="World stats">
    <ignition-gui>
        <title>World stats</title>
        <property type="bool" key="showTitleBar">false</property>
        <property type="bool" key="resizable">false</property>
        <property type="double" key="height">110</property>
        <property type="double" key="width">290</property>
        <property type="double" key="z">1</property>

        <property type="string" key="state">floating</property>
        <anchors target="3D View">
        <line own="right" target="right"/>
        <line own="bottom" target="bottom"/>
        </anchors>
    </ignition-gui>

    <sim_time>true</sim_time>
    <real_time>true</real_time>
    <real_time_factor>true</real_time_factor>
    <iterations>true</iterations>
    <topic>/world/world_demo/stats</topic>

</plugin>
```
### Explaining XML: Add light to Gazebo world

```xml
<light type="directional" name="sun">
	    <cast_shadows>true</cast_shadows>
	    <pose>0 0 10 0 0 0</pose>
	    <diffuse>0.8 0.8 0.8 1</diffuse>
	    <specular>0.2 0.2 0.2 1</specular>
	    <attenuation>
		<range>1000</range>
		<constant>0.9</constant>
		<linear>0.01</linear>
		<quadratic>0.001</quadratic>
	    </attenuation>
	    <direction>-0.5 0.1 -0.9</direction>
	</light>
```

### Explaining XML: Add Physics 

```xml
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
```
### Run .SDF file


* Run the world: [Click to view entire XML code](../units/Unit01-ROS-2-Basics/L02-Gazebo-World/code/L02-S01-empty_world.sdf)

```
ign gazebo empty_world.sdf
```


![Empty Gazebo World](../media/empty_gazebo.png)

### Add shapes and models to your empty Gazebo World

* For the empty world, add a simple ground plane model with neccesary link
```xml
 <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
      <pose>0 0 0 0 -0 0</pose>
    </model>
```
### add Gravity, magnetic field and atmosphere

* Run the world: [Click to view entire XML code](../units/Unit01-ROS-2-Basics/L02-Gazebo-World/code/L02-S02-model_shape.sdf)

```xml
  <gravity>0 0 -9.8</gravity>
  <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
  <atmosphere type='adiabatic'/>
```

### Add Objects to the gazebo world

* After launching Ignition GUI. In the GUI, the top left toolbar hanging below the file menu button contains transform control (first four buttons) and shape buttons (sphere, box, cylinder).

```
ign gazebo empty_world.sdf
```

* Add shapes and try changing the color of the objects 

* Let say you spawned a box on to your world and you would like to change the color of the box. Then, select on the box and follow below steps:
  * In your right side of the panel, expand the box and select the box_visual option
  * Under visual, select Material and change the diffuse color.

![Gazebo World with Shapes](../media/shapes_world.png)

### Add models from Fuel

* Instead of building our own models we can use already built ones. Ignition Fuel hosts hundreds of models that can easily be added to an Ignition world. Refer the [tutorial](https://gazebosim.org/docs/citadel/fuel_insert) for detailed explanation.

![Husky model on Fuel](../media/husky_fuel.png)

### Tutorial on adding Fuel model to the world

* Goto [Fuel](https://app.gazebosim.org/fuel), let say you would like to invoke Spot robot into your gazebo world. Under model section choose your desired model by copying "SDF snippet". In this example, visit [Husky_Robot](https://app.gazebosim.org/OpenRobotics/fuel/models/MARBLE_HUSKY_SENSOR_CONFIG_5) model.

![Husky model on Fuel](../media/husky.png)

### Tutorial on adding Fuel model to the world 

* Once you have your model, copy the snippet to your clip board and paste it onto your SDF file under world tag <world> ... </world> and launch your ROS 2 application

```
<include>
<uri>
https://fuel.gazebosim.org/1.0/OpenRobotics/models/csiro_data61_spot_sensor_config_1
</uri>
</include>
```

![Husky model on Gazebo World](../media/husky1.png)

### Tutorial on adding Fuel model to the world 

* Add shapes and obstacles to Gazebo World from top left toolbar

![Husky with Box Obstacle](../media/husky_objects.png)

### Husky says Hi to TB4 -  Fun Fact!!!

![TurtleBot 4 says Hi!](../media/husky_tb4.png)

### Spawn a TurtleBot4 on to Gazebo world

* To launch turtle bot 4 with warehouse world, below follow below steps. The turtlebot4_ignition_bringup package contains launch files and configurations to launch Ignition Gazebo.

```
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable 
`lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```
```
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ignition-edifice ros-galactic-turtlebot4-simulator ros-galactic-irobot-create-nodes
```

### Locally Spawn a TurtleBot4 on to Gazebo world 

* Create a new workspace with a src folder in it and colcon build the workspace

```
mkdir turtle_ws
cd turtle_ws && mkdir src
colcon build
```
* clone the Turtlebot4 Simulator

```
cd turtle_ws/src
git clone https://github.com/turtlebot/turtlebot4_simulator.git
cd .. && colcon build
```
### Locally Spawn a TurtleBot4 on to Gazebo world 

* Source the package

```
. install/setup.bash
```
* Launch TurtleBot4

```
ros2 launch turtlebot4_ignition_bringup ignition.launch.py
```
![TurtleBot 4 says Hi!](../media/tb4_warehouse.png)

### Spawn a TurtleBot4 on to custom Gazebo world

* Create a Custom world as per your requirement and used case. Below is the sample SDF comprising of empty warehouse world. Add world from fuel by copying the snippet shown below.

```
cd src/turtlebot4_simulator/turtlebot4_ignition_bringup/worlds
touch custom_world.sdf
```

```xml
<include>
  <uri>
    https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_WallB_01
  </uri>
</include>
```

* [Click to view entire XML code](../units/Unit01-ROS-2-Basics/L02-Gazebo-World/code/L02-S05-custom_world.sdf)

### Spawn a TurtleBot4 on to custom Gazebo world 


* Launch the TurtleBot4 on to your custom world

```
colcon build
ros2 launch ignition.launch.py world:=custom_world
```

![TurtleBot 4](../media/tb4_0.png)

### Spawn a TurtleBot4 on to custom Gazebo world

![TurtleBot 4](../media/tb4_1.png)