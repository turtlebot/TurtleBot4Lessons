template: ../media/TB4Template.pptx

# Objective: Unit 1 Lesson 1 Topics and Transfer 

* Understand ROS messages
* Understand the concept of a publish / subscribe systems.
* Understand ROS middlewares.
* Identify ROS namespaces. 
* Understand and use the ros CLI for `ros2 topic`.

## ROS Messages 

![A stamp](https://upload.wikimedia.org/wikipedia/commons/thumb/2/26/Iranian_postage_stamp.jpg/360px-Iranian_postage_stamp.jpg)

* A key feature of ROS is its message system. 
* ROS Messages are like a standard sized mail envelope. They hold a value.
* A message is simply a collection of data, arranged as a set of fields with a `name` and `type`. 
  * The position of a robot, or an image are examples of a message.
  * Messages can be composed from other messages, e.g. a robot's state message may be composed of a position message and an orientation message. 
  * These messages are similar to Python's `dict` type or C++'s `struct`.
* Each ROS package can define a set of messages or use messages defined in another package. 
* ROS has **many** prefined messages for common robot component. 
  * These shared messages are what make ROS so powerful! 
    
## ROS Messages in Practices

![A ROS Message](https://raw.githubusercontent.com/osrf/TurtleBot4Lessons/main/media/rosmsg.png)

* Let's look at an example from TB4. 
* Message files use the `*.msg` suffix. Let's look at a few. 
* Left pane: We used `find` to list all of the `*.msg` files in the TB4 sources. 
* Right pane: We used `less` to view the `UserLed.msg` message files. 
  * An LED is a light emitting diode, a small light bulb.
* We can see that messages are simply types (e.g. `uint8`) and a name. There are also _comments_ telling us what the messages represent.
* The `UserLed` message defines which `led` you want to color and the `color` you want to make it.

## Moving Messages

![Mail Truck](https://upload.wikimedia.org/wikipedia/commons/thumb/0/02/USPS-Mail-Truck.jpg/640px-USPS-Mail-Truck.jpg)

* In the abstract, ROS moves messages using a concept called `publish / subscribe`, often abbreviated `pub/sub`.
* It is easier to think of publishing as sending, and subscribing as recieving. 
* In ROS messages are published and subscribed to `topics`. 
* `Topics` are like a phone number, or a mail box, they allow the same _message type_ to be used _multiple times_.
* ROS topics are _hierarchical_, they can be grouped in a logical hierarchy. 
* This hierarchy is denoted using the forward slash, "/", similarly to your computer's hard disk. 

## Where our Analogy Fails

![Broadcast](https://upload.wikimedia.org/wikipedia/commons/thumb/7/75/Laika_ac_Sutro_Tower_%287078966285%29.jpg/640px-Laika_ac_Sutro_Tower_%287078966285%29.jpg)

* Pub/sub systems aren't exactly like the mail in some respects. In some ways they're more like broadcast towers! 
* In the mail, a letter goes from exactly one house to exactly one other house. We call this one-to-one transfer. 
* In a pub/sub system messages are moved in a many-to-many fashion.
* In a pub/sub system multiple people can send and recieve messages to a topic.
* When a program _subscribes_ to a topic it is asking to recieve messages from a topic.
* When a program _publishes_ to a topic it is asking to send messages to **everyone** subscribed to a topic. 
* In some ways a topic is like a public TV station, where anyone can make a program and anyone can watch that show (if they want). 

## How does ROS Move Messages?

![Transport](https://raw.githubusercontent.com/osrf/TurtleBot4Lessons/main/media/transport.png)

* ROS doesn't actually move messages published to a topic! 
* Instead, ROS has a generic interface for moving messages, and you choose how you want them to be moved. 
* This generic layer is called a "ROS Middleware Layer" or `RMW`. 
* An RMW is like an adapter that let's you plug ROS into lots of different software protocols and hardware transport layers.
  * If you want to move ROS to publish topics over carrier pigeon or e-mail it is possible. 
* Most ROS 2 robots use `DDS` as their RMW. DDS stands for _Data Distribution Service._ 
* [DDS is a generic protocol for moving data.](https://en.wikipedia.org/wiki/Data_Distribution_Service). 
   * DDS can supports a variety of transport protocols like WiFi, 4G, 5G, and even USB.  
* There are a number of vendors of DDS software. Some of them are open-source and others are proprietary. 


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
