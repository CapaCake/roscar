#!/bin/bash
source ~/.bashrc
cur_dateTime=$(date "+%Y%_m%_d%H%M%S")
echo ${cur_dateTime}
rosrun map_server map_saver -f  /home/racecar/smartcar/src/racecar/map/${cur_dateTime}
kill -9 $(pgrep rviz)
kill -9 $(pgrep gmapping)
