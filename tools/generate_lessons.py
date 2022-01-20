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
    path = os.path.dirname(os.path.abspath(tutorials_file))
    for tut in tutorials["tutorials"]:
        to_process = os.path.join(path, tut["file"])
        if os.path.exists(to_process):
            tut["input_fname"] = to_process
            tut["output_fname"] = to_process[:-3]+".ppt"
        else:
            print("Can't find file {0}".format(to_process))
            retval = False

    path = os.path.dirname(os.path.abspath(units_file))
    all_lessons = []
    # Check the lessons and units
    for unit in units["units"]:
        lesson_yml = os.path.join(path,unit["directory"],unit["lessons"])
        
        if os.path.exists(lesson_yml):
            with open(lesson_yml) as f:
                try:
                    lesson_info = yaml.load(f, Loader=yaml.FullLoader)
                    # For each lesson
                    for lesson in lesson_info["lessons"]:
                        temp = {}
                        # check data is all there
                        temp["unit"] = unit["number"]
                        temp["unit_name"] = lesson_info["name"]
                        #temp["unit_file"] = os.path.join(lesson_path,
                        temp.update(lesson)

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

    return retval, all_lessons, tutorials

def process_ppt(file_list):
    result = True
    root_path  = "../units/"
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



        
    return True

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--units", help="List of all units to generate", type=str, default="../units/curriculum.yml")
    parser.add_argument("--tutorials", help="Tutorials yaml file", type=str, default="../tutorials/tutorials.yml")
    args = parser.parse_args()
    result, lessons, tutorials = parse_and_verify(args.units, args.tutorials)
    if not result: 
        print("Failed to parse input files.")
        exit(1)
    result = process_ppt(lessons)
    if not result: 
        print("Failed to generate slides.")
        exit(1)

    print("Processing completed successfully!")


if __name__ == "__main__":
    main()
