template: ../media/TB4Template.pptx

# Unit 0, Lesson 1: Ignition Blastoff

### Lesson Objectives 

* Learn what a simulation is in the context of robotics.
* Learn what Ignition is and where it came from. 
* Select the appropriate installation procedure your computer
* Setup the Ignition and TurtleBot4 simulation for Ignition
* Understand the steps involved in the setup process
* Learn basic controls for both Ignition and TurtleBot. 



### What is a Simulation?

![The Matrix](https://upload.wikimedia.org/wikipedia/commons/a/a8/Matrix-plano_subjetivo.jpg?20211216120933)

* A simulation is a virtual world that includes a subset of features from the real world. 
* In the context of robotics, simulations are virtual worlds that allow roboticists to be more productive.
* Simulations are very similar to modern video games in that they include realistic physics simulations, a notion of time, and graphical representations of the real world.
* A second way to think of a simulation is a robot's imagination. Your robot can experience a particular scenario and try different approaches to solve a problem. 
* Simulations are also a productivity tool for roboticists; making it easier to perform a variety of tasks. 

### Why Do We Use Simulation?

![DARPA SubT](https://upload.wikimedia.org/wikipedia/commons/thumb/3/35/SubT-logo_centered_color-MAIN.png/638px-SubT-logo_centered_color-MAIN.png)

* It is often said that robots are best suited to dull, dirty, and dangerous tasks, and by extension so is the process of developing a robot. 
* A simulation allows a robot developer to increase their productivity in a variety of ways.
  * With a simulation you can create a arbitrary number of robots and put them an arbitrary number of scenarios, without worrying about costs. 
  * Working on robots can get dirty. Simulations allow us to work from our desks.
  * Robots can be dangerous! It is much safer to test your algorithms in simulation before applying them to a real robot. 
  * Simulation is cost effective. Building a gaming computer is much cheaper than building a robot from scratch.
* Simulation has become ubiquitous in the field of robotics. Competitions like DARPA SubT, NIST ARIAC, VRX, and many others now use simulation for robotics competitions. 


### What is Ignition?

![Ignition Logo](https://ignitionrobotics.org/assets/images/logos/ignition_logo_color_horizontal.svg)

* Ignition, is a collection of simulation software maintained by the same group that maintains ROS, Open Robotics.
* At its core Ignition consists of game engine, a physics engine, a plug-in library, and the infrastructure necessary to interface ROS.
* Ignition is latest incarnation of the Gazebo simulator. Ignition is to Gazebo as ROS 1 is to ROS 2
* The Ignition simulator is called Ignition Gazebo, while the other software tools are referred to as Ignition.
* Ignition supports a number of infrastructure tools that support developers:
  * [Ignition Fuel](https://app.ignitionrobotics.org/fuel) -- A collection of drag-and-drop 3D assets and environments for scenario development.
  * [Cloud Sim](https://gitlab.com/ignitionrobotics/web/cloudsim) -- A tool for running simulations on a cloud instance
  * [SDFormat](http://sdformat.org/spec) -- A specification for creating rich simulation elements.

### Installing Ignition Fortress


![Ignition Fortess Logo](https://raw.githubusercontent.com/osrf/TurtleBot4Lessons/main/media/fortress.jpg?token=GHSAT0AAAAAABQJBI4RQAB5ABAKPJV47E3QYQVLMFA)

* Just like ROS, Ignition has a yearly release where odd year releases are considered long term support (LTS) releases. 
* The TurtleBot4 Simulation use the Ignition Fortress LTS release.
* [The full installtion notes for Ignition Fortress can be found on the Ignition Website.](https://raw.githubusercontent.com/osrf/TurtleBot4Lessons/main/media/fortress.jpg?token=GHSAT0AAAAAABQJBI4RQAB5ABAKPJV47E3QYQVLMFA)
* Ignition, like ROS 2, is free and open source software, and you should be able to run it on most systems, but that doesn't mean it works perfectly on every computer! 
* Due to frequent changes in the software libraries used in Ignition, not every operating system will have binary packages for Download.
* If your host OS is listed in the installation notes you should be able to install Fortress natively; otherwise we recommend that you install Fortress in a virtual machine. 

### Installing Ignition Fortress II

| Host OS |Recommended Installation |Configuration|Note|
| Ubuntu 20.04   | [Standard Install](https://ignitionrobotics.org/docs/fortress/install_ubuntu)          | Binary / Apt    | **Recommended **              |
| Ubuntu < 20.04 | [VM with Ubuntu 20.04](https://www.virtualbox.org/wiki/Downloads)                      | VM              | ROS 2 H/G Requires 20.04      |
| Other Linux    | [VM with Ubuntu 20.04](https://www.virtualbox.org/wiki/Downloads)                      | VM              | Binary installation possible  |
| Windows 10     | [Standard Windows Install](https://ignitionrobotics.org/docs/fortress/install_windows) | Binary / Conda  | Ubuntu VM is alternative      |
| Other Windows  | [VM with Ubuntu 20.04](https://www.virtualbox.org/wiki/Downloads)                      | VM              |                               |
| macOS          | [VM with Ubuntu 20.04](https://www.virtualbox.org/wiki/Downloads)                      | VM              | Ignition will work, ROS won’t |

## Installing A Virtual Machine

![VM](https://upload.wikimedia.org/wikipedia/commons/a/a2/VirtualBox_Guest_Additions_Logo_2010.png)

* As you can see from the chart above most host operating sytems will require a virtual machine. 
* While ROS and Igntion *can* run on a wide variety of operating systems many of them require compilation from source code.
  * New users often run in to difficulties compiling from source. This is why we recommend a virtual machine. 
* While a container is also a possible solution we recommend a VM as they are usually easier to navigate for new users. 
* We recommend VirtualBox, but VMWare or any other VM client should work. 
* The steps for VM installation are as follows:
  * [Install the VM Client for your host system.](https://www.virtualbox.org/wiki/Downloads)
  * [Create a new Virtual Machine and Install Ubuntu 20.04](https://linuxhint.com/install_ubuntu_virtualbox_2004/)
  * [Start the VM and follow Ubuntu 20.04 Installation](https://ignitionrobotics.org/docs/fortress/install_ubuntu)

## Installing Ignition

* For the rest of the lesson we'll assume you are running Ubuntu 20.04. 
* The next step is to install the Ignition Fortress binaries.
* Installation process is simple:
  * Install tools
  * Add GPG key for apt repositories
  * Update package list
  * Install Binaires
* Apt is the Ubuntu installation tool, it is like an app-store for Linux.
* Open a terminal and run the following commands

```
sudo apt-get install lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
	signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
	http://packages.osrfoundation.org/gazebo/ubuntu-stable \
	$(lsb_release -cs) main" | \
	sudo tee /etc/apt/sources.list.d/gazebo-stable.list \
	> /dev/null
sudo apt-get update
sudo apt-get install ignition-fortress
```
## Fire up Ignition

* If everything installed correctly you can now start Ignition.
* Open a terminal 


## ROS Discourse

![Discourse](https://raw.githubusercontent.com/osrf/TurtleBot4Lessons/main/media/discourse.png?token=GHSAT0AAAAAABQJBI4R3NBN6D6RVUL6XO5EYQMGQ7Q)


* [ROS Discourse](http://discourse.ros.org) is the official ROS discussion forum. 
* ROS Discourse is intended for community announcements and discussion.
  * **ROS Discourse should not be used for specific development questions.**
* On ROS Discourse you find.
  * Announcements of new packages and package releases.
  * Announcements of ROS releases and updates.
  * Technical discussions by package developers. 
  * Event announcements.
  * Regular news updates. 
  * Meet other ROS developers and package maintainers. 


# Turtle Bot 4 -- A Fancy Robot

* Let's learn about robots!

## This Is A Presentation Section Page

![A TurtleBot 4](../media/tb4.png)

* Today we will learn how to use a TurtleBot 4

### This Is A Bulleted List on TB4 Resources

* One
  * One A
  * One B
* Two
   * foo
   * bar
   * baz
   
** Here are some slide notes. Note you leave an empty line between the content - in this case a bulleted list - and the notes. **


## Here is a page with some code!

* For help search [Google](http://www.google.com). 
* `This is highlighted code`
** This is emphasis code. ** 

```
      sudo apt update
      sudo apt install -y python3-rosdep
      sudo rosdep init
      rosdep update
```

* This is some text describing above

### Here is another page

* Herp
* Derp
* Slerp
* Perp

** WILL THIS TEXT SHOW UP? **

## How about an image and code 

![A TurtleBot 4](../media/tb4.png)

```
      sudo apt update
      sudo apt install -y python3-rosdep
      sudo rosdep init
      rosdep update
```

## How about an image and text???

#### This slide is an image and text! This text won't appear

* This text will appear!
* So will this
* So will this?

![A TurtleBot 4](../media/tb4.png)

### Let's try a different way

* This text will appear!
* So will this
* So will this?

![A TurtleBot 4](../media/tb4.png)



### What about an H4 with a wall of text

**Here is some subtext**

#### What happens with an H4

Lorem ipsum dolor sit amet, consectetur adipiscing elit. Integer ultrices posuere lorem, eu dictum massa efficitur ac. Aenean at risus luctus, vulputate ex non, condimentum tellus. Aenean egestas, odio eu ornare eleifend, nulla enim efficitur nunc, non luctus velit purus a urna. Pellentesque magna odio, viverra eu enim non, hendrerit pulvinar nibh. Aenean blandit lacus et ex bibendum, at finibus orci rhoncus. Integer venenatis eros in scelerisque porta. Nam ligula sem, ullamcorper nec efficitur in, commodo sit amet justo. Praesent ut nibh ultrices, euismod eros in, consectetur risus. Fusce a imperdiet justo. Cras magna massa, venenatis non lectus ac, mattis luctus est.

Phasellus varius rutrum ullamcorper. Maecenas iaculis rhoncus ipsum, id congue augue suscipit eu. Nullam varius neque non est congue tincidunt. Vestibulum a nulla ac felis consequat posuere. Ut vulputate malesuada placerat. Integer accumsan rhoncus magna sit amet laoreet. Nullam aliquet libero pharetra, suscipit felis at, faucibus enim. Sed metus massa, tincidunt vitae nunc quis, bibendum laoreet ante. Donec id egestas dui. Quisque blandit tellus augue, in cursus mauris malesuada eget.

## How about two images?

![A TurtleBot 4](../media/tb4.png)
![A TurtleBot 4](../media/tb4.png)

## What about tables?

| a | b | c | d | e |
|---|---|---|---|---|
| 1 | 2 | 3 | 3 | 4 |
| a | f | g | d | s |
| s | s | s | s | s |


__This is a underlined under a table__

# Do we get syntax highlighting

```
    def sortedNumericList(string):
        return sorted(list(map(int, set(string.split()))))

    def parseRGB(str):
        if RGBmatch := RGBRegex.match(str):
            # Matches
            return (True, RGBmatch.group(1))
        else:
            return (False, "")
```
