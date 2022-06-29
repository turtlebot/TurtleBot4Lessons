# Please Note

** This repository is a work in progress, the lessons may be incomplete or unfinished. **




# TurtleBot4Lessons

This repository contains classroom friendly lessons, tutorials, projects, and questions, and materials for the TurtleBot 4 (TB4).

[The full index of units and lessons is available here.](./index.md)


# General Layout

```
Root
  Media (Images, etc)
  Tutorials (stand alone lessons, e.g. docker, vm, etc)
    Tutorial000-Name
      TutorialContent
  Units
    Unit00-Name
      Unit00-Name-Lesson00-Name
        Unit00-Name-Lesson00-Name
	src
      Unit00-Name-Project
        Unit00-Name-Project.md
	src
      Unit00-Name-QuestionBank
  Tools  
```



# Classroom Slides

Each TB4 lesson comes with an instructional Markdown file that can be easily converted into a Google Slides Deck or Microsoft Power Point slide deck. Instructors are encoraged to create, update and modify these slides as they see fit. For convience we have include the PowerPoint Slide and *a link to a read-only copy of the slides in Google Slides*.

## Generating Slides

Slides are generated using a utility called [md2pptx](https://github.com/MartinPacker/md2pptx). Md2pptx has some pecularities about its syntax but it is generally proficient at converting markdown notes to slides. These instructions for installation are tested and verified on Ubuntu 20.04.

Install the dependencies 

```
pip3 install python-pptx
```

In the root of this directory clone md2pptx

```
git clone git@github.com:MartinPacker/md2pptx.git
cd md2pptx
sudo chmod +x md2pptx
<TODO> system level install 
```

Md2pptx uses a template PowerPoint file as the basis for its slides. We've built a default TB4 template and it is located at <TODO>. You are more than welcome to modify this template as you see fit.

# Generating Slides on Older Versions of Ubuntu (e.g. 18.04)

md2pptx needs Python 3.8+ to run correctly, and on older systems you may encounter errors such as, "if RGBmatch := RGBRegex.match(str):", if this is the case you should use a virtual environment to run the script. 

First install Python 3.8:

``` bash
$ sudo apt-get install python3.8 python3.8-dev python3.8-distutils python3.8-venv
```

Next, go to the root of the directory, and create your virtual environment, activate it

``` bash
$ cd TurtleBot4Lessons 
$ python3.8 -m venv env
$ source env/bin/activate
```

Check that you got the right version of python using:

``` python
which python

```

Next we'll install all of our Python 3.8 dependencies 

``` bash

python3.8 -m pip install wheel 
python3.8 -m pip install python-pptx pyyaml

```

Now install the md2pptx script, copy over the executable, and set it to executable. 

``` bash
cd ./TurtleBot4Lessons
git clone git@github.com:MartinPacker/md2pptx.git
cp md2pptx/md2pptx ./tools/
sudo chmod +x ./md2pptx/md2pptx
```


Now you should be able to generate the lessons correctly. 

``` bash
cd tools
python3.8 generate_lessons.py 

```

Next time you want to run the lesson script, just activate the virtual env and be on your merry way. 

``` bash
source ./TurtleBot4Lessons/env/bin/activate
```




# Viewing Slides

When creating slides it is helpful to see text and images a they render. The best solution I have found so far is to use a small python application called [grip](https://github.com/joeyespo/grip). 

## Installation on Linux 

```
sudo pip install grip
```

## Execution on Linux 

```
grip -b index.md

```
# Creating Tables 

To generate markdown tables use [CSV2MD](https://github.com/lzakharov/csv2md). Please keep a local CSV file. 


## Helpful Links

* [Free Wikimedia Images](https://commons.wikimedia.org/wiki/Main_Page)
* [Umich Robotics 501: Mathematics for Robotics](https://github.com/michiganrobotics/rob501)
