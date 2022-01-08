import argparse
import yaml
import os.path

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
                        temp.update(lesson)
                        ppt_file = os.path.join(path,unit["directory"],
                                                lesson["directory"],lesson["ppt-file"])
                        # Verify the file exists
                        if os.path.exists(ppt_file):
                            all_lessons.append(temp)
                        else:
                            print("Failed to find: {0}".format(ppt_file))
                            retval = False
                            
                except Exception as e:
                    print("Error encountered {0}".format(e))
                    retval = False

    return retval, all_lessons

def process_ppt(file_list):
    pass

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--units", help="List of all units to generate", type=str, default="../units/curriculum.yml")
    parser.add_argument("--tutorials", help="Tutorials yaml file", type=str, default="../tutorials/tutorials.yml")
    args = parser.parse_args()
    result, info = parse_and_verify(args.units, args.tutorials)
    if not result: 
        print("Failed to parse input files.")
        exit(1)

    print("Processing completed successfully!")


if __name__ == "__main__":
    main()
