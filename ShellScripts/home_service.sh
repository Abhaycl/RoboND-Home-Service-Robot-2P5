#!/bin/sh
export TURTLEBOT_3D_SENSOR=kinect
catkin_src_dir=/home/workspace/catkin_ws/src

# Launch turtlebot in the custom world
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$catkin_src_dir/World/MyWorld.world" &
sleep 6

# Launch amcl demo
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$catkin_src_dir/World/my_map.yaml 3d_sensor:=kinect" &
sleep 5

# Launch rviz
xterm -e "rosrun rviz rviz -d $catkin_src_dir/RvizConfig/home_service.rviz" &
sleep 10

# Launch markers add_markers node
xterm -e "rosrun add_markers add_markers" &
sleep 5

# Launch navigation pick_objects node
xterm -e "rosrun pick_objects pick_objects"