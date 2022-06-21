template: ../media/TB4Template.pptx

# Objective: Unit 1 Lesson 2 Section 2

## Add shapes and models to your empty Gazebo World

1. For the empty world, add a simple ground plane model with neccesary link

```xml
<sdf version='1.8'>
  <world name='empty_world_shapes'>
    <scene>
      <ambient>1 1 1 1</ambient>
      <background>0.8 0.8 0.8 1</background>
      <shadows>1</shadows>
    </scene>
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
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
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
  </world>
</sdf>
```

2. After launching Ignition GUI. In the GUI, the top left toolbar hanging below the file menu button contains transform control (first four buttons) and shape buttons (sphere, box, cylinder).

```
ign gazebo empty_world.sdf
```

3. Add shapes and try changing the color of the objects 

- Let say you spawned a box on to your world and you would like to change the color of the box. Then, select on the box and follow below steps:
	- In your right side of the pannel, expand the box and select the box_visual option
	- Under visual, select Material and change the diffuse color.

![Gazebo World with Shapes](../media/shapes_world.png)
