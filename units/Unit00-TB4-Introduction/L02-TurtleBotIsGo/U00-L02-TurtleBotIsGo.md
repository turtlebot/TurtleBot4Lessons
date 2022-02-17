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

![wifi setup](https://raw.githubusercontent.com/osrf/TurtleBot4Lessons/main/media/wifisetup.png?token=GHSAT0AAAAAABQJBI4QI7XHQS57XDSAKJEYYQW42SA)

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

![Create Wifi Setup](https://raw.githubusercontent.com/osrf/TurtleBot4Lessons/main/media/createwifisetup.png?token=GHSAT0AAAAAABQJBI4RVPHFBHP27J6JS2VIYQW5HAA)

* TurtleBot has not one, but two computers on board and each of them needs to be connected to wifi.
* This procedure is used to connect the Create3 to your wireless network.
* Press both button 1 and 2 on the Create3 simultaneously until the light ring turns blue.
* The Create3 is now in AP mode, connect to its WiFi `Create-XXXX`
- In your web browser go to `192.168.10.1`
- Click connect and enter your wifi SSID and password
- Wait for it to connect to wifi, the ring light will turn white. 


  
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
