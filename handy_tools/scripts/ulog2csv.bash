#!/bin/bash
# Convert to CSV a set of ULOG files. If the folder path is not set, current path will be used
# Usage:
# ./ulog2csv.bash path_to_ULG_files_folder
#
# Install dependencies:
# pip install pyulog

# Set minimum size filter to select the ULG files to be converted. Default: 0 MB
MIN_SIZE=4M

# Only consider to get the given messages. Comment this line to get all messages available in the ULG file
MSG_FILTER=vehicle_attitude,vehicle_attitude_setpoint,vehicle_local_position,vehicle_local_position_setpoint

if [ -z "$1" ]
then
     DIR="."
fi

echo $(find $DIR -name '*.ulg' -size +$MIN_SIZE)

for ulg_file in $(find $DIR -name '*.ulg' -size +$MIN_SIZE); do 
     current_pwd="${ulg_file::-4}"
     mkdir -p "$current_pwd"
     if [ -z "$var" ]
     then
          ulog2csv -o $current_pwd -m $MSG_FILTER $ulg_file
     else
          ulog2csv -o $current_pwd $ulg_file
     fi     
     # Uncomment this line to add the ULG file to the CSVs folder
     # mv $ulg_file $current_pwd
done
