template: ../media/TB4Template.pptx

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

## Choose The Right Tool

* Why does the difference between source and binaries matter?
  * Because the difference between the two dictates how you install it on a robot. 
* Moreover, there are lots of tools and services that are needed to install software on your robot.
* Much like Python, ROS has a variety of tools and services that help you:
  * Build binaries from source code. 
  * Install binaries that were compiled somewhere else.
  * Mix your source code, with other source code, with binaries. 
  * Keep track of everything that is going on!
* If ROS seems complex, that's because it is. There is a lot going on under the hood.

## Operating Systems 

![ROS and Linux Distros](https://upload.wikimedia.org/wikipedia/commons/8/89/Logo_Collage_Linux_Distro.png)

* An operating system is what makes your computer do what you want, and there are a lot of them.
* A distro, or *distribution*, is a specific version of software
  * Some folks also call a distro a release. 
  * These are often given both a code name and a number.
	* These numbers usually denote either a number in a series or a release date. 
	* e.g. **Jammy Jellyfish 22.04** or **ROS 2 Humble Hawksbill**

## ROS Distros

![rolling](../media/rolling.png)

* ROS distributions are released yearly on May 23rd, [World Turtle Day.](https://www.worldturtleday.org/)
* Even year releases are considered _long term support_ (LTS) distributions.
  * ROS 2 Foxy Fitzroy and Humble Hawksbill are LTS Releases. 
  * This means they recieve regular updates and bug fixes for a set period of time, usually three to five years. 
  * *Most users should use the latest LTS version of ROS.*
* Odd year releases are only supported for a short period of time (example: ROS 2 Galactic).
* ROS 2 also has what we call a "rolling" release, called Rolling Ridley
  * A rolling release is simply a regular release using the latest ROS code available. 
  * The regular ROS releases are a well tested clone of the rolling release. 


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
