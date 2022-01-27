 # TurtleBot4Lessons

This repository contains classroom friendly lessons, tutorials, projects, and questions, and materials for the TurtleBot 4 (TB4).

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


# Creating Slides
