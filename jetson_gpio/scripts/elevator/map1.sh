#!/bin/bash
{
gnome-terminal -x bash -c "roslaunch hanning_navigation localizatiom_map1.launch"
}&


wait
exit 0
