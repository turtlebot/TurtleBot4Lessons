template: ../media/TB4Template.pptx

# Objective: Unit 1 Lesson 2 Section 3

## Add custom models from [Fuel](https://app.gazebosim.org/fuel)
Instead of building our own models we can use already built ones. Ignition Fuel hosts hundreds of models that can easily be added to an Ignition world. Refer the [tutorial](https://gazebosim.org/docs/citadel/fuel_insert) for detailed explanation.

#### Example

- Goto [Fuel](https://app.gazebosim.org/fuel), let say you would like to invoke Spot robot into your gazebo world. Under model section choose your desired model by copying "SDF snippet". In this example, visit [Husky_Robot](https://app.gazebosim.org/OpenRobotics/fuel/models/MARBLE_HUSKY_SENSOR_CONFIG_5) model.

![Husky model on Fuel](../media/husky.png)

- Once you have your model, copy the snippet to your clip board and paste it onto your SDF file under world tag <world> ... </world> and launch your ROS 2 application

```
<include>
<uri>
https://fuel.gazebosim.org/1.0/OpenRobotics/models/csiro_data61_spot_sensor_config_1
</uri>
</include>
```

- The output of your gazebo world should look like below

![Husky model on Gazebo World](../media/husky1.png)

- Add shapes and obstacles to Gazebo World

![Husky with Box Obstacle](../media/husky_objects.png)

- Fun Fact!!!

![TurtleBot 4 says Hi!](../media/husky_tb4.png)
