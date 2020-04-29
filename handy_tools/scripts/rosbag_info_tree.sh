#!/usr/bin/env bash
# Script to execute rosbag info of a directory tree which contains rosbag files
# Output saved in text file
# Original code in https://github.com/kddeisz/tree

dir_count=0
file_count=0
index=0
traverse() {
  dir_count=$(expr $dir_count + 1)
  local directory=$1
  local prefix=$2

  local children=($(ls $directory))
  local child_count=${#children[@]}

  for idx in "${!children[@]}"; do 
    local child="${children[$idx]}"
    local child_prefix="│   "
    local pointer="├── "

    if [ $idx -eq $(expr ${#children[@]} - 1) ]; then
      pointer="└── "
      child_prefix="    "
    fi

    echo -n "${prefix}${pointer}$child";
    if [ ${child: -4} == ".bag" ]; then 
      index=$((index+1))
      echo " ($index)"
      echo "($index) $child ()" >>  "./out.txt"
      echo "" >>  "./out.txt"
      rosbag info "$directory/$child" >> "./out.txt"
      echo "" >>  "./out.txt"
    else
      echo ""
    fi

    [ -d "$directory/$child" ] &&
      traverse "$directory/$child" "${prefix}$child_prefix" $index ||
      file_count=$(expr $file_count + 1)
  done
}

root="."
[ "$#" -ne 0 ] && root="$1"
echo $root

traverse $root ""
echo
echo "$(expr $dir_count - 1) directories, $file_count files, $index rosbags"
