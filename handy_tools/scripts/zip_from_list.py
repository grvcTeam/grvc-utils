#!/usr/bin/env python
from zipfile import ZipFile
import argparse
import datetime
import rospkg
import yaml
import glob
import os

def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description='Zip a collection of files from a config file list')
    parser.add_argument('-zip', type=str, default="",
                        help='name of the zip output file')
    parser.add_argument('-list', type=str, default="",
                        help='name of the yaml input config file list')
    args, unknown = parser.parse_known_args()
    # utils.check_unknown_args(unknown)

    if args.zip:
        zip_url = args.zip
        if zip_url[-4:] != '.zip':
            zip_url += '.zip'
    else:
        now = datetime.datetime.now()
        zip_url = zip_url = os.getenv("HOME") + '/zipped_' + now.strftime("%Y%m%d_%H%M%S") + '.zip'

    if args.list:
        list_url = args.list
    else:
        # Get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        list_url = rospack.get_path("handy_tools") + "/config/zip_from_list.yaml"

    with open(list_url, 'r') as list_file:
        yaml_data = yaml.load(list_file)

    print "Elements of list_to_zip in [{0}] will be zipped to [{1}]".format(list_url, zip_url)

    success_count = 0
    fail_count = 0
    with ZipFile(zip_url,'w') as zip: 
        for element in yaml_data["list_to_zip"]:
            element = os.path.expanduser(element)
            element = os.path.expandvars(element)
            expanded_element = glob.glob(element)
            if not expanded_element:
                print "Unable to find file or files [{0}]".format(element)
                fail_count += 1
            for finally_a_file in expanded_element:
                print "Zipping file [{0}]".format(finally_a_file)
                zip.write(finally_a_file)
                success_count += 1
    print "{0} files successfully zipped, {1} failures".format(success_count, fail_count)

if __name__ == "__main__": 
    main() 
