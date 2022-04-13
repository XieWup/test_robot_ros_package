#!/bin/bash

{
gnome-terminal -x bash -c "roslaunch hanning_navigation localizatiom_map2.launch"
}&

wait
exit 0
