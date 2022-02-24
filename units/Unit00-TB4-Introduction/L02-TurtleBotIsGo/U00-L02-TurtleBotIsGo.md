template: ../media/TB4Template.pptx

### Lesson Objectives 

* Familiarize yourself with the TurtleBot 4 
* Connect the TurtleBot 4 to your local wifi.
* Update the TurtleBot 4
* Connect to the TurtleBot
* Run the teleop launch files.

## The TurtleBot 4

![A TurtleBot 4](../media/tb4.png) ![A TurtleBot 4 Lite](../media/tb4.png)

* There are two variants of the TurtleBot 4. 
* TODO: DIAGRAMS
* Turtle Bot 4 includes:
  * [iRobot Create 3](https://edu.irobot.com/what-we-offer/create3)
  * [RaspberryPi 4b - 4GB](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/)
  * [OAK-D Pro](https://docs.luxonis.com/projects/hardware/en/latest/pages/DM9098pro.html)
  * [RPLidar](https://www.slamtec.com/en/Lidar/A1) 
  * Circuit Board
* TurtleBot4 Lite
  * [iRobot Create 3](https://edu.irobot.com/what-we-offer/create3)
  * [RaspberryPi 4b - 4GB](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/)
  * [RPLidar](https://www.slamtec.com/en/Lidar/A1) 
  * [OAK-D Lite](https://docs.luxonis.com/projects/hardware/en/latest/pages/DM9095.html)
  
## Setting Up Your TurtleBot -- Connect to Wifi

![wifi setup](https://raw.githubusercontent.com/osrf/TurtleBot4Lessons/main/media/wifisetup.png?token=GHSAT0AAAAAABR5R4E3L4LYJHVOB7JINGAEYRBFXUA)

* Plug in dock, and place robot on dock to power it on.
* It will take a few minutes for the robot to boot.
* Using a Linxu PC to connect to RPI in your wifi settings:
	* SSID: `Turtlebot4` Password: `Turtlebot4`
* SSH into robot from the same laptop: `ssh ubuntu@10.42.0.1`
  * Password: `turtlebot4`
* In the home directory there is a script for setting up the WiFi `wifi.sh`
* Call `bash wifi.sh -s "wifi_ssid" -p "wifi_password" -c REG_DOMAIN && sudo reboot`
* `REG_DOMAIN` depends on your country, [see this list.](https://www.arubanetworks.com/techdocs/InstantWenger_Mobile/Advanced/Content/Instant%20User%20Guide%20-%20volumes/Country_Codes_List.htm#regulatory_domain_3737302751_1017918) 
* USA: `US`, Canada: `CA`, UK: `GB`, Germany: `DE`, Japan: `JP3`, Spain: `ES`.

## Now Setup the Create3 Wifi

![Create Wifi Setup](https://raw.githubusercontent.com/osrf/TurtleBot4Lessons/main/media/createwifisetup.png?token=GHSAT0AAAAAABR5R4E3RZJC47Q6ZC23QYLAYRBFYKA)

* TurtleBot has not one, but two computers on board and each of them needs to be connected to wifi.
* This procedure is used to connect the Create3 to your wireless network.
* Press both button 1 and 2 on the Create3 simultaneously until the light ring turns blue.
* The Create3 is now in AP mode, connect to its WiFi `Create-XXXX`
* In your web browser go to `192.168.10.1`
* Click connect and enter your wifi SSID and password
* Wait for it to connect to wifi, the ring light will turn white. 

## Check Your Installation

![Internal Nodes](https://raw.githubusercontent.com/osrf/TurtleBot4Lessons/main/media/internalnodes.png?token=GHSAT0AAAAAABQJBI4R7I26QTKBUNZN7QCWYQ6YAZQ)

* Give your TB4 a minute to save its configuration.
* Reboot the Turtlebot4 by pressing <TODO>
* The TB4 should have a white ring light and play a tone when it has started. 
* Now find your TurtleBot's IP address <TODO> 
* Open a terminal and SSH into the robot using: `ssh ubuntu@<TB4 IP>`
* The password is `turtlebot4`
* Let's check that everything is connected to the network, run the command
  * `ros2 node list`
* You should see the `_internal/*` nodes.
  * If these nodes are not visible then the Create3 is not connected to the network. 
	
## Setting up Teleoperation

![PS4 BLE Setup](https://raw.githubusercontent.com/osrf/TurtleBot4Lessons/main/media/PS4BLE.png?token=GHSAT0AAAAAABQJBI4QQ3NFHHJPEZRLHZ44YQ6YS6Q)

* Now let's connect to the robot and put it in teleoperation.
* For this you'll need a bluetooth PS4 controller. Make sure it is charged!
* ssh into the robot and run the following commands to enable bluetooth pairing.
  * `sudo bluetoothctl`
  * `default-agent`
  * `scan on`
  * After entering scan on the controller will respond, "Discovery started"
* Now we'll put the controller into pairing mode by pressing and briefly holding the two buttons shown on the diagram.
* The front light will start blinking rapidly twice in a row. 
* You should see something like `[NEW] Device D8:E8:DD:8A:B9:30 <some string>`
* At some point you should see something like `[NEW] Device D8:E8:DD:8A:B9:30 Wireless Controller`

## Pairing the Bluetooth Controller

![PS4 BLE Setup](https://raw.githubusercontent.com/osrf/TurtleBot4Lessons/main/media/PS4BLE.png?token=GHSAT0AAAAAABQJBI4QQ3NFHHJPEZRLHZ44YQ6YS6Q)

* Once you see the controller copy its MAC address. It is the series of numbers of letters separated by colons.
* Now run the following where <MAC> is your controller's unique MAC address. 
  * `trust <MAC>`
  * `pair <MAC>`
  * `connect <MAC>`
* You have now paired your bluetooth controller. You can exit the controller by calling `exit`
* On subsequent connections you won't need to run through every step. Instead you can run:
  * Check the controller is running with:
  * `ubuntu@ubuntu:~/turtlebot4_ws$ sudo bluetoothctl paired-devices`
	* ` Device A5:15:66:C1:AC:46 Wireless Controller`
  * If the device is there you're good to go. 
  * Otherwise follow the steps above. 


## Multiple Terminals with Byobu

![Byobu](https://raw.githubusercontent.com/osrf/TurtleBot4Lessons/main/media/byobu.png?token=GHSAT0AAAAAABR5R4E27LUFIF3BWXC5K4MCYRBFUUA)

* Before we put the robot into teleoperation mode, we need to review how to open multiple terminals in an SSH session.
* There are many terminal managers, also called multiplexers available for Linux. Examples include [Byobu](https://www.byobu.org/), [Tmux](https://www.ocf.berkeley.edu/~ckuehl/tmux/), and [GNU Screen](https://www.gnu.org/software/screen/).
* For our examples, we'll use Byobu, but you can use whatever you're comfortable with! It is also worth noting that byobu generally uses tmux as its backend. 
* To start byobu simply type `byobu` in your terminal.
  * If byobu is not installed, simply run `sudo apt install byobu`.
* To create a new terminal simply press `F2`.
* To page through terminals use `F3` and `F4`.
* `shift-F2` - Split the screen horizontally and `ctrl-F2`  Split the screen vertically.
* `shift-F3` - Shift the focus to the previous and `shift-F4` Shift to the next split .
* `ctrl-F6` - Remove this split
* `ctrl-F5` - Reconnect GPG and SSH sockets
* [Full documentation can be found here.](https://www.byobu.org/documentation) or by running `man byobu`, or `shift-F1` in Byobu.

# Running Teleoperation
	
* ROS 2 uses "launch" files to run collections of small programs called nodes.
* We're going to run a launch file called "joy_teleop.launch.py", where "joy_teleop" means "joystick teleoperation"
* Create a terminal in your robot ssh session and run the following:
  * `cd ~/turtlebot4_ws`
	* Move to our ROS 2 "workspace."
  * `source ./install/setup.bash`
	* Tell the terminal we're using this ROS 2 workspace. 
  * `ros2 launch turtlebot4_bringup joy_teleop.launch.py `
	* Run the program `joy_teleop.launch.py` found in the directory` ~/turtlebot4_ws/src/turtlebot4/turtlebot4_bringup/launch/`,
  * A bunch of stuff should appear on the screen.
  * Note that the `tab` key should automatically complete most of these commands after you enter the first few letters. 


# Moving the TB4 in Teleoperation Mode 

* The TurtleBot4 controller operates as a "dead man's switch."
* A "dead man's" switch is a safety feature that is often used with robot controllers. The user must hold down the "dead man switch" at all times; this prevents the robot from moving if the controller is dropped or misplaced.
* For this controller "L", the button near your left index finger, is the deadman switch.
* There is a second dead man switch, the "R" button near your right index finger. This switch makes the robot move faster. 
* The left joystick controls the robot's direction. 
* The robot is smart, and will temporarily disable the controller if hits a cliff or a big bump. It may also do this if you reverse suddenly. This mode is indicated by the ring light turning yellow. 

  
  
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
