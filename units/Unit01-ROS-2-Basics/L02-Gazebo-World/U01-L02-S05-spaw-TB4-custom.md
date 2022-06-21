template: ../media/TB4Template.pptx

# Objective: Unit 1 Lesson 2 Section 5

## Spaw a turtle Bot 4 onto custom world

```
cd src/turtlebot4_simulator/turtlebot4_ignition_bringup/worlds
touch custom_world.sdf
```

- Create a Custom world as per your requirement and used case. Below is the sample SDF comprising of empty warehouse world.


```xml

<?xml version="1.0"?>
<sdf version="1.8">
  <world name="empty_warehouse">
    <scene>
      <grid>false</grid>
    </scene>
    <physics name="1ms" type="ignored">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"></plugin>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"></plugin>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"></plugin>


    <include>
      <uri>
        https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_WallB_01
      </uri>
    </include>
    
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
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.90000000000000002</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
  </world>
</sdf>

```
- Launch the Turtle Bot 4 on to your Custom world

```
colcon build
ros2 launch ignition.launch.py world:=custom_world
```

![TurtleBot 4](../media/tb4_0.png)
![TurtleBot 4](../media/tb4_1.png)