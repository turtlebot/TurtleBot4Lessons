template: ../media/TB4Template.pptx

# Unit 1 Lesson 2 Create Empty Gazebo World 

### Objective

* Understand Simulation with Ignition Gazebo and ROS 2
* How to create an empty Gazebo world
* How to add a objects to Gazebo world
* How to spawn a TurtleBot4 in a Gazebo world 
* Understanding Launch files
* Creating your own Gazebo visual assets

### Create Empty Gazebo World

* To create an empty world, please follow this tutorial on the [Gazebo_docs](https://gazebosim.org/docs/citadel/sdf_worlds). 
* This tutorial will introduce you to () SDF, how to create an empty world, and how to add models to it.
* This tutorial also explains all the Gazebo Plugins required for creating your a custom simulation world.

![Empty Gazebo World](../media/empty_gazebo.png)

### Creating an Empty World

* The first step in creating your own Gazebo world is to create an empty SDF file.
* This empty SDF file will hold all of the elements in your simulation.
* We'll be following this [Gazebo_docs](https://gazebosim.org/docs/citadel/sdf_worlds) to create an empty SDF file. 
* We'll create the empty file

    * Create an empty `.sdf` file using the `touch` command
	* Then edit the new file using the `gedit` command. Copy the content from `./code/L02-S01-empty-world.sdf`. 
	* Finally, run the Ignition simulation contained in your SDF file. 

```
touch empty_world.sdf
gedit empty_world.sdf 
ign gazebo empty.sdf

```

### Your First World

* If you've done everything correctly Gazebo should launch and you should see an empty world like the one below. 
* Feel free to look around the world to see what we've created. 
* Close the simulation using the X in the top right 

![Empty Gazebo World](../media/empty_gazebo.png)


### Explaining XML and the XDF File Format

* Internally, SDF files use the XML specification to define a Gazebo world.
* XML is similar to HTML where there are tag elements that have both opening and closing tags. 
* Opening tags look like: `<tag>` and closing tags look like `</tag>`
* Each SDF file starts with an xml version tag, an SDF version tag, and a world name.
* The closing tags are at the end of the file. 

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="empty_world">
  ...
  </world>
</sdf>
```
### Explaining XML: Add Plugins

* SDF and Gazebo let you use `plugins` to add functionality to your simulation.
* In your XML file you must specify the plugin filename and the plugin name.
* For our simulation we'll be including the Physics, User Commands, and Scene Broadcaster plugins. 
* Here is an example of what is in your "empty" world SDF file. 

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

### Explaining XML: Add Gazebo GUI 


* Now let's define our Gazebo GUI. 
* Under the <gui> tag we specify anything related to the GUI of Ignition. 
* [ignition-gui](https://github.com/gazebosim/gz-gui) has a bunch of plugins to choose from. 
* We will add the ones that are necessary to get our world up and running with basic functionality.

```xml
<gui fullscreen="0">
  .
  .
  .
	
</gui>

```
### Explaining XML: Add Gazebo GUI- Scene 3D plugin 

* Next we'll add a scene 3D plugin to GUI. 
* The GzScene3D plugin is responsible for displaying the 3D scene of our world.
* This plugin let's us manually configure a variety of aspects of the Gazebo GUI including the GUI state, the ambient lighting, the camera position, and the background color. 


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



###  Explaining XML: Add GUI- World Control Plugin 

* The World control plugin is responsible for controlling our view of the Gazebo world.
* This plugin enables things like the play_pause button, the stats_topic, start_paused
* This plugin includes general properties like, the height and width of the window. 

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
### Explaining XML: World Stats Plugin 

* The World Stats Plugin is responsible for displaying the simulation world statistics, these includes  `<sim_time>`, `<real_time>`, `<real_time_factor>` and `<iterations>`.
* These numbers tell you how the total elapsed time in simulation, total real time elapsed, the real time factor of the simulation, and the number of simulation iterations. 
* Like before, the plugin includes generic properties like width, height, and resizeable. 

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
### Explaining XML: Add Light to our Gazebo World

* The next step in our SDF file is to create a "Sun" object so we can see our environment. 
* To create our sun we'll create a single source of light using the `light` XML tag. 
* The light object below specifies a single point light source in our simulation. 

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

* Another crucial component of a Gazebo simulation is physics simulation.
* In the following block we set the physics engine (ODE), the real-time factor (the "speed" of the simulation), and the maximum time step size. 

```xml
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
```
### Run .SDF file


* Now that we've walked through our "empty" Gazebo world let's restart it and take a look. 
* [The full SDF file can be found here](../units/Unit01-ROS-2-Basics/L02-Gazebo-World/code/L02-S01-empty_world.sdf)
* You can load Gazebo using the following command:

```
ign gazebo L02-S01-empty_world.sdf
```

![Empty Gazebo World](../media/empty_gazebo.png)

### Adding a Ground Plane to our Empty World

* Let's modify our empty world to add a ground plane a few simple shapes. 
* The first thing we're going to add to our empty world is a `ground plane` upon which our robot will sit. 
* We will use the `model` tag to load a simple model into our world. 
* Each SDF `model` has two  parts:
  * Collision -- how other objects will interact with the world.
  * Visual -- how the object will look on the screen. 
* The `geometry` for our model will be a simple plane with a size of 100x100 with a normal pointing up. 



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
```


### Adding a Ground Plane to our Empty World (Cont.)

* The visual component of our floor model has two components:
  * The geometry, in this case a plane of size 100 x 100 with a normal pointing up.
  * The material, in this a case a plain gray material..

```
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
### Finally, add Gravity, Magnetic Field and Atmosphere

* The last thing we need to do is add some basic physics parameters. 
* The first parameter is gravity, which we'll set to 9.8m/s^2 in the negative Z direction. 
* Since our robots might have a magnetometer we're also going to add a magnetic field to mimic the earth.
* Finally, we'll set the atmosphere type to `adiabatic`.

```xml
  <gravity>0 0 -9.8</gravity>
  <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
  <atmosphere type='adiabatic'/>
```

# Run Our Ground Plane World

* Now that our world is set up, let's run it. 
* [The complete file can be found at code/L02-S02-model_shape.sdf](../units/Unit01-ROS-2-Basics/L02-Gazebo-World/code/L02-S02-model_shape.sdf)
* You can run the world just like the previous SDF files, using the command:

```
ign gazebo L02-S02-model_shape.sdf
```


### View your Ground Plane World


![Ground Plane World](../media/ground_plane.png)


### Adding Simple Objects to a Gazebo World 

* After launching Ignition, in the GUI, on the top left toolbar hanging below the file menu button contains transform control (first four buttons) and shape buttons (sphere, box, cylinder).
* You can click on any of the shapes to add them to the Gazebo World
* Add shapes and try changing the color of the objects 
* If you would like to change the color of any of the objects you have added, simply select the objects you have created and follow below steps:
  * In your right side of the panel, expand the box and select the box_visual option
  * Under visual, select Material and change the diffuse color.

![Gazebo World with Shapes](../media/shapes_world.png)

### Add models from Fuel

* Instead of building our own models we can use already built ones. 
* Ignition Fuel hosts hundreds of models that can easily be added to an Ignition world.
* Next we're going to add a robot or object model from the internet. 
* Refer the [tutorial](https://gazebosim.org/docs/citadel/fuel_insert) for detailed explanation.
* TODO: upgrade to Fortress and summarize tutorials. 


![Husky model on Fuel](../media/husky_fuel.png)

### Tutorial on adding Fuel model to the world

* Next we're going to add a model from Fuel to our Gazebo simulation. 
* First go to the fuel website: [https://app.gazebosim.org/fuel](https://app.gazebosim.org/fuel)
* Under model section choose your desired model by copying "SDF snippet". In this example, visit [Husky_Robot](https://app.gazebosim.org/OpenRobotics/fuel/models/MARBLE_HUSKY_SENSOR_CONFIG_5) model.
* TODO: SHOW SDF Snippet on Fuel


![Husky model on Fuel](../media/husky.png)

### Adding a Fuel Model to the World 

* Once you have your model, copy the snippet to your clip board and paste it into your SDF file under world tag `<world> ... </world>`
* & launch your ROS 2 application TODO: What does this mean
* TODO: this URI does not match the code 
* TODO: This needs to be launched, where does the code live. 
```
<include>
<uri>
https://fuel.gazebosim.org/1.0/OpenRobotics/models/csiro_data61_spot_sensor_config_1
</uri>
</include>
```

### Feel Free to Add Shapes to Your Downloaded Models
 
![Husky with Box Obstacle](../media/husky_objects.png)


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

### Spawn a TurtleBot4 in a Custom Gazebo World

* Finally we'll create a custom world for our Turtlebot4. 
* Below is the sample SDF comprising of empty warehouse world that we've pulled from Ignition Fuel 
* You can add the world from Fuel by copying the snippet shown below:

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




































### Spawn a TurtleBot4 on to Gazebo world

* To launch turtle bot 4 with warehouse world, below follow below steps. The turtlebot4_ignition_bringup package contains launch files and configurations to launch Ignition Gazebo.
* TODO: We do not need to have installation instructions



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
