template: ../media/TB4Template.pptx

# Unit 0, Lesson 3: Finding Help

### Lesson Objectives 

* Remember that robotics is hard. You will get stuck. You will make mistakes.
* Being able to quickly find help for your problems is crucial skill. 
* The objective for this lesson is to introduce the resources available for ROS.
* You should bookmark all of these websites in your browser. 


## Help Others Help You

![Binary](https://upload.wikimedia.org/wikipedia/commons/thumb/1/11/Question_in_a_question_in_a_question_in_a_question.gif/328px-Question_in_a_question_in_a_question_in_a_question.gif)


* Writing a good question is a critical to getting a good response.
* Generally speaking, the shorter the question, the less likely it is to be answered. 
* Longer, more complete questions, that provide all the necessary information, allow others to more easily help you. 
* A good ROS question should contain:
  * The body of question with as much detail as possible. 
  * General details about your system, such as host operating system, the version of ROS, and what packages you have installed.
  * The error message in its entirety. This should be quoted text and not a screenshot,
  * References to documentation, resources, and other questions you have tried but did not work. 
* Distinguish between a discussion questions, poorly scoped questions, and legitimate errors and issues. 
  * A discussion question is one where there is no single correct answer.
    * Questions like, "What is the best...", or "How should I make X?" are discussion questions
  * A poorly scoped question is one where the response could be multiple paragraphs long. These sort of questions need to broken down into smaller parts.
    * Questions like, "How do I build a delta robot in ROS?" or "How do I build a perception pipeline to find X?" are poorly scoped.
  * Legitimate errors and issues are usually easily described. 
    * Questions like, "When I do X, Y error occurs" or "When I use X, I expect Y but get Z, why is this?"

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
