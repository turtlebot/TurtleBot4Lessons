import argparse
import yaml
import os.path
import os
import subprocess

def check_yaml(fname):
    retval = None
    status = True
    if not os.path.exists(fname):
        print("{0} -- file could not be found".format(fname))
        status = False
    else:
        with open(fname) as f:
            try:
                retval = yaml.load(f, Loader=yaml.FullLoader)
            except Exception as e:
                print("Error encountered {0}".format(e))
                status = False
    return status, retval


def parse_and_verify(units_file, tutorials_file):
    retval = True

    status, units = check_yaml(units_file)
    if not status:
        print("Can't find or verify {0}".format(units_file))
        return False

    status, tutorials = check_yaml(tutorials_file)
    if not status:
        print("Can't find or verify {0}".format(units_file))
        return False

    # Check the tutorials
    all_tutorials = [] 
    path = os.path.dirname(os.path.abspath(tutorials_file))
    for tut in tutorials["tutorials"]:
        to_process = os.path.join(path, tut["file"])
        if os.path.exists(to_process):
            tut["input_fname"] = to_process
            tut["output_fname"] = to_process[:-3]+".ppt"
            all_tutorials.append(tut)
        else:
            print("Can't find file {0}".format(to_process))
            retval = False

    path = os.path.dirname(os.path.abspath(units_file))
    all_lessons = []
    
    # Check the lessons and units

    md_index = []
    md_index.append( "# TurtleBot 4 Syllabus \n\n")

    md_path = "./"+os.path.relpath(path , "../")
    

    for unit in units["units"]:
        lesson_yml = os.path.join(path,unit["directory"],unit["lessons"])
        
        if os.path.exists(lesson_yml):
            with open(lesson_yml) as f:
                try:
                    lesson_info = yaml.load(f, Loader=yaml.FullLoader)
                    # For each lesson
                    mdl = "* Unit {0}: {1}\n".format(lesson_info["unit"], lesson_info["name"] )
                    md_index.append(mdl)
                    mdl = "\t* [Question Bank]({0})\n".format("B")
                    md_index.append(mdl)
                    mdl = "\t* [Project]({0})\n".format("A")
                    md_index.append(mdl)
                    for lesson in lesson_info["lessons"]:
                        temp = {}
                        # check data is all there
                        temp["unit"] = unit["number"]
                        temp["unit_name"] = lesson_info["name"]                        
                        #temp["unit_file"] = os.path.join(lesson_path,
                        temp.update(lesson)

                        # lp = lesson path, power point and md path relative to github
                        lp = os.path.join(md_path, unit["directory"],
                                          lesson["directory"])
                        lp_ppt = os.path.join(lp,lesson["ppt-file"])
                        # TODO ppt-file is a bad name
                        lp_md = os.path.join(lp,lesson["ppt-file"].replace(".md",".ppt"))
                                          
                        
                        mdl = "\t* Unit {0}: [{1}]({2})-[PPT]({3})\n".format(lesson["number"],
                                                                    lesson["name"],
                                                                    lp_md,
                                                                    lp_ppt)
                        md_index.append(mdl)

                        # TODO: "ppt-file is bad name, fix it, perhaps source_file
                        ppt_file = os.path.join(path,unit["directory"],
                                                lesson["directory"],lesson["ppt-file"])
                        # Verify the file exists
                        if os.path.exists(ppt_file):

                            # Generate the paths to make life easier
                            temp["full-path"] = ppt_file
                            temp["target-file"] = lesson["ppt-file"].split(".")[0] + ".ppt"
                            temp["target-file-full-path"] = os.path.join(path,unit["directory"],
                                                lesson["directory"],temp["target-file"])
                            all_lessons.append(temp)
                        else:
                            print("Failed to find: {0}".format(ppt_file))
                            retval = False
                            
                except Exception as e:
                    print("Error encountered {0}".format(e))
                    retval = False

    return retval, all_lessons, all_tutorials, md_index

def process_lessons(file_list):
    result = True
    for lesson in file_list:
        print("Processing unit {0} - lesson {1} - {2} from file {3}".format(
            lesson["unit"],lesson["number"],lesson["name"],lesson["ppt-file"]))
        print("Converting\nFrom:{0}\nTo  :{1}\n\n".format(lesson["full-path"],lesson["target-file-full-path"]))
        cli_call = ["./md2pptx", "{0} < {1}".format(lesson["target-file-full-path"],lesson["full-path"])]
        print("Executing "+ " ".join(cli_call))
        os.system(" ".join(cli_call))
        # TODO, subprocess is hanging for some reason
        # but I would really like to 
        #subprocess.call(cli_call)

    return result 

def process_tutorials(file_list):
    result = True

    md_index = []
    md_index.append( "# Tutorials\n\n")
    
    for tutorial in file_list:
        print(tutorial)
        print("Processing tutorial: {0}".format(tutorial["name"]))
        print("Converting\nFrom:{0}\nTo  :{1}\n\n".format(tutorial["input_fname"],tutorial["output_fname"]))
        cli_call = ["./md2pptx", "{0} < {1}".format(tutorial["output_fname"],tutorial["input_fname"])]
        print("Executing "+ " ".join(cli_call))
        os.system(" ".join(cli_call))
        # TODO: Change to subprocess
        
        # Create a markdown index
        markdown_entry = "* [{0}]({1}) -- [PPT]({2})\n"
        md_p =  "./"+os.path.relpath(tutorial["input_fname"],"..")
        ppt_p =  "./"+os.path.relpath(tutorial["output_fname"],"..")
        temp = markdown_entry.format(tutorial["name"],md_p,ppt_p)
        md_index.append(temp)
       
    return result, md_index 


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--units", help="List of all units to generate", type=str, default="../units/curriculum.yml")
    parser.add_argument("--tutorials", help="Tutorials yaml file", type=str, default="../tutorials/tutorials.yml")
    args = parser.parse_args()
    result, lessons, tutorials, unit_index  = parse_and_verify(args.units, args.tutorials)
    if not result: 
        print("Failed to parse input files.")
        exit(1)

    result = process_lessons(lessons)
    if not result: 
        print("Failed to generate slides.")
        exit(1)

    result, tutorial_index = process_tutorials(tutorials)
    if not result: 
        print("Failed to generate tutorials.")
        exit(1)

    index_file = "../index.md"
    print("Writing tutorial index at {0}".format(index_file))
    with open(index_file,"w") as fp:
        for t in tutorial_index:
            fp.write(t)
        fp.write("\n\n")
        for t in unit_index:
            fp.write(t)
        

        

    print("Processing completed successfully!")


if __name__ == "__main__":
    main()
