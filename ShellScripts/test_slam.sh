#!/bin/sh
export TURTLEBOT_3D_SENSOR=kinect
catkin_src_dir=/home/workspace/catkin_ws/src

# Launch turtlebot in the custom world
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$catkin_src_dir/World/MyWorld.world" &
sleep 6

# Launch gmapping demo
xterm -e "roslaunch turtlebot_navigation gmapping_demo.launch" &
sleep 6

# Launch turtlebot teleop
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch" &
sleep 6

# Launch rviz
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch"