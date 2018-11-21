#!/bin/sh

# Launch gazebo
xterm -e "gazebo" &
sleep 5

# Launch the ROS master
xterm -e "source /opt/ros/kinetic/setup.bash; roscore" &
sleep 5

# Launch rviz
xterm -e "rosrun rviz rviz"