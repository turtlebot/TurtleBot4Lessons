import argparse
import yaml
import os.path

def check_yaml(fname):
    retval = None
    status = True
    if not os.path.exists(fname):
        print( "{0} -- file could not be found".format(fname))
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

    status, units = check_yaml(units_file)
    if not status:
        return False

    status, tutorials = check_yaml(tutorials_file)
    if not status:
        return False
    
    path = os.path.dirname(os.path.abspath(tutorials_file))
    for tut in tutorials["tutorials"]:
        to_process = os.path.join(path, tut["file"])
        if os.path.exists(to_process):
            tut["input_fname"] = to_process
            tut["output_fname"] = to_process[:-3]+".ppt"
        else:
            print("ooops")

    path = os.path.dirname(os.path.abspath(units_file))
    for unit in units["units"]:
        unit_yml = os.path.exists(os.path.join(unit["directory"],unit["lessons"]))
        # if os.path.exists(unit_yml):
        #     with open(unit_yml) as f:
        #         try:
        #             unit_info = yaml.load(f, Loader=yaml.FullLoader)
        #             print(unit_info)
        #         except Exception as e:
        #             print("Error encountered {0}".format(e))
        # else:
        #     print("oops 2")

    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--units", help="List of all units to generate", type=str, default="../units/curriculum.yml")
    parser.add_argument("--tutorials", help="Tutorials yaml file", type=str, default="../tutorials/tutorials.yml")
    args = parser.parse_args()
    if not parse_and_verify(args.units, args.tutorials):
        print("Failed to parse input files.")
        exit(1)

    print("Processing completed successfully!")


if __name__ == "__main__":
    main()
