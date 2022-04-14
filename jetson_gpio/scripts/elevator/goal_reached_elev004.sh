#!/bin/bash
source ~/robot_ws/devel/setup.bash
rosrun jetson_gpio goal_reached_elev004.py &
echo "goal_reached_elev004.py starting success!"
wait
exit 0
