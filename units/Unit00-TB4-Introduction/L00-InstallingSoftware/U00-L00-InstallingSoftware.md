template: ../media/TB4Template.pptx

template: ../media/TB4Template.pptx

# Unit 0, Lesson 1: GazeboSim

### Lesson Objectives 

* Remember that robotics is hard. You will get stuck. You will make mistakes.
* Being able to quickly find help for your problems is crucial skill. 
* The objective for this lesson is to introduce the resources available for ROS.
* You should bookmark all of these websites in your browser. 


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


# Unit 0, Lesson 0: Setting up Software

### Lesson Objectives 

* Understand the different ways we can install software.
* Understand ROS and Linux Distros.
* Understand the different versions of ROS
* Select the correct ROS distro and correct installation mechanism for your system
* Finally, setup the TurtleBot 4 and host laptop.



## Software Packaging 

![Binary](https://upload.wikimedia.org/wikipedia/commons/f/fb/Basic_Idea_of_a_Compiler.png)

* Generally speaking there are two ways that software can arrive at "your computer" and the way this software arrives dictates how it is installed. Most of this is review, but a refresher is helpful for understanding ROS, and Linux.
* *Pre-Compiled Binaries* -- often called binaries, are software programs that have be *compiled* into objects that are ready to run on your computer. 
  * These binaries have two main types:
	* Libraries -- sections of code that are reused over and over, like common math functions. 
	* Executables -- also called programs are pieces of code that do some task and then end. 
  * Most executables, or programs, will have a main part that then makes use of multiple libraries.
  * Many libraries are already installed on your computer, but some you may need to download and install. 
* *Source Code* -- source code, or just code, is written in a human readable language that is compiled (i.e. translated) into executables. 
  * Open Source Code is code that is freely available and shareable. You'll find it on places like Github or Gitlab.
  * Closed Source Code is code that is only shared via binaries, that you must often pay for.
* There are many exceptions to this rule, programming languages like Python sit somewhere in between the two. Languages like C and C++ have a clear distinction between binaries and source code.

### Binaries vs. Source Code

* Why does the difference between source and binaries matter?
* Because the difference between the two dictates how you install it on a robot.
* ROS has utilities to:
  * Build binaries from source code. 
  * Install binaries that were compiled somewhere else.
  * Mix your source code, with other source code, with binaries. o
  * Keep track of everything that is going on!

## Operating Systems 

![ROS and Linux Distros](https://upload.wikimedia.org/wikipedia/commons/8/89/Logo_Collage_Linux_Distro.png)

* An operating system is what makes your computer do what you want, and there are a lot of them.
* A distro, or *distribution*, is a specific version of software
  * Some folks also call a distro a release. 
  * These are often given both a code name and a number.
	* These numbers usually denote either a number in a series or a release date. 
	* e.g. **Jammy Jellyfish 22.04** or **ROS 2 Humble Hawksbill**

## ROS Distros

![ROS and Linux Distros](https://raw.githubusercontent.com/ros-infrastructure/artwork/master/distributions/rolling/rolling.png)

* ROS distributions are released yearly on May 23rd, [World Turtle Day.](https://www.worldturtleday.org/)
* Even year releases are considered _long term support_ (LTS) distributions.
  * ROS 2 Foxy Fitzroy and Humble Hawksbill are LTS Releases. 
  * This means they receive regular updates and bug fixes for a set period of time, usually three to five years. 
  * *Most users should use the latest LTS version of ROS.*
* Odd year releases are only supported for a short period of time (example: ROS 2 Galactic).
* ROS 2 also has what we call a "rolling" release, called Rolling Ridley
  * A rolling release is simply a regular release using the latest ROS code available. 
  * The regular ROS releases are a well tested clone of the rolling release. 

## Tier 1 Operating Systems 

![Operating Systems](https://upload.wikimedia.org/wikipedia/commons/1/1d/Cartoon_Hands_Opening_A_Video_Sharing_Site_In_A_Laptop.svg)

* Each ROS distro is designed to work well on a handful of **specific** operating systems.
* These specific operating systems are called **Tier 1** operating systems.
* A full list of Tier 1 operating systems can be found in [REP-2000](https://www.ros.org/reps/rep-2000.html#humble-hawksbill-may-2022-may-2027)
  * REP-2000 provides other information like:
	  * How to install ROS on your host machine (i.e. source, binary, or package manager).
	  * What combinations of OS and architecture (e.g. arm vs amd)are supported
	  * Specific software libraries that are required.
* Tier 2 and Tier 3 operating systems for a particular distro may work but are not regularly tested.
* ROS is FOSS, but that doesn't mean every OS / ROS Distro combo will work well.
* **Your life will be significantly easier if you use the correct ROS Distro for your OS**

## Quickly Connecting to Your TB4

![VM Diagram](https://upload.wikimedia.org/wikipedia/commons/9/90/OpenBSD_starting_SSH_server.jpg)
* **If your goal is simply to start your TurtleBot, configure it, and run a few programs on the robot you may not need to install ROS on your laptop.**
* You can use a secure shell protocol, called `ssh`, to connect your robot. 
* [SSH](https://en.wikipedia.org/wiki/Secure_Shell) is a simple protocol to connect one computer to another computer, perhaps one running on a robot.
* To connect to the robot you will need to install an SSH client on your laptop or desktop operating system.
* Your TB4 and your laptop will need to be on the same network, and you will need the TurtleBot's IP address. 
* Setting up an SSH connection to your TB4 is covered under [Tutorial X: How to SSH into a TurtleBot 4](TODO). 



### Laptops and Robots 

![Operating Systems](https://upload.wikimedia.org/wikipedia/commons/b/b8/Cart_pushing_rviz_holonomic.jpg)

* Up until now we've been talking about the OS and ROS Distro on a robot, like the TB4.
* When you're using, programming, or debugging a robot you'll need to interact with the robot's code. 
* Depending on what you are doing, your laptop may need to run ROS too! 
  * If you plan to tele-operate (remote control) your robot or visualize its data your laptop will need to run ROS!
  * You may also want to visualize data from your robot
	* ROS 2 has a program called RVIZ for visualizing robot data.
   * Sometimes computation can be complex! You may want to do those computations not on the robot. 
   * In these three cases your laptop may need to run ROS too!
* **Things to Keep Mind**
  * If two or more computers are using ROS in robot, then they must use the same ROS Distro!
  * You cannot connect two different ROS distros together!
  * Your best bet is to have your laptop and robot run the same OS and ROS Version!
* There are work arounds if this isn't feasible.


### Connecting Robots and Laptops 
 

* Up until now we've been talking about the OS and ROS Distro on a robot, like the TB4.
* When you're using, programming, or debugging a robot you'll need to interact with the robot's code. 
* Depending on what you are doing, your laptop may need to run ROS too! 
* If you plan to tele-operate (remote control) your robot or visualize its data your laptop will need to run ROS!
* You may also want to visualize data from your robot
  * ROS 2 has a program called RVIZ for visualizing robot data.
* Sometimes computation can be complex! You may want to do those computations not on the robot. 
* In these three cases your laptop may need to run ROS too!
* **If two or more computers are using ROS in robot, then they must use the same ROS Distro**
* Ordinarily, you can not connect two different ROS distros together!


### Virtual Machines, Containers, and ROS

![VM Diagram](https://upload.wikimedia.org/wikipedia/commons/0/08/Hardware_Virtualization_%28copy%29.svg)

* **What if I don't want to change my operating system to use ROS?**
* There are two ways to get around the requirement that your host OS match the preferred host OS. 
* Virtual Machines
  * A [virtual machine (VM)](https://en.wikipedia.org/wiki/Virtual_machine) emulates a second computer on your host computer. 
  * There are lots of different VM software providers. Some examples are Virtual Box and VMWare
  * Virtual machines can display the full GUI of the desired operating system and are usually easier for those just getting familiar with the technology. 
  * Installing ROS on a VM is covered in our [Virtual Machine Tutorial.](TODO)
* Containers
  * [Containers](https://en.wikipedia.org/wiki/OS-level_virtualization) are similar to VMs in that they allow your computer to emulate a different operating system. Containers, however, are "stripped down" versions of VMs that have lower overhead.
  * Common containerization software include Docker, Singularity, and Podman, with Docker being the most common. 
  * Containers are generally easier to share, modify, and distribute than VMs. 
  * Installing ROS on a Docker container is covered in our [Docker Tutorial.](TODO) 

## Setting up your Desktop / Laptop


| Use Case                 | Advantage                          | Disadvantage                              | Tutorial                           |
|--------------------------|------------------------------------|-------------------------------------------|------------------------------------|
| SSH into TB4             | Simple                             | Limited Features                          | [How to setup SSH](TODO)           |
| Install VM on Laptop     | Easy to run graphical applications | VM may be resource constrained            | [Installing ROS on a VM](TODO)     |
| Install Docker on Laptop | Best for advanced developers       | Graphical applications may require rocker | [Installing ROS with Docker](TODO) |
| Full Linux Installation  | Best overall experience            | (None)                                    | [Installing ROS on Linux](TODO)    |


## Installation Directions by Host OS

| Laptop OS                  | Virtual Machine | Docker   | Native Support                  |
|----------------------------|-----------------|----------|---------------------------------|
| Ubuntu 20.04 (Focal Fossa) | Possible        | Possible | X (BEST CHOICE)                 |
| Other Ubuntu               | Recommended     | Possible | Not Recommended                 |
| macOS                      | Recommended     | Possible | Not Recommended                 |
| Windows 10                 | Recommended     | Possible | Possible*                       |
| Linux Variants             | Recommended     | Possible | Possible for RHEL 8 and Debian  |

* Configurations listed as possible may require additional commands and configuration. 
* ROS can be installed on many operating systems from source but is not recommended for new users. 
* Take home message: the recommended configuration for most operating systems is to create an Ubuntu 20.04 Virtual Machine

### Connecting Robots and Laptops 
 

* **How do I run software on my TurtleBot4?** 
 * SSH 
 * Virtual Machines

#### Let's try this
* Up until now we've been talking about the OS and ROS Distro on a robot, like the TB4.
* When you're using, programming, or debugging a robot you'll need to interact with the robot's code. 
* Depending on what you are doing, your laptop may need to run ROS too! 
* If you plan to tele-operate (remote control) your robot or visualize its data your laptop will need to run ROS!
* You may also want to visualize data from your robot
  * ROS 2 has a program called RVIZ for visualizing robot data.
* Sometimes computation can be complex! You may want to do those computations not on the robot. 
* In these three cases your laptop may need to run ROS too!
* **If two or more computers are using ROS in robot, then they must use the same ROS Distro**
* `Ordinarily, you can not connect two different ROS distros together!`



## This Is A Presentation Section Page

![A TurtleBot 4](../../../media/tb4.png)

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
