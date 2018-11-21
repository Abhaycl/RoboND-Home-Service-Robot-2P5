# Deep RL Arm Manipulation Project Starter Code

The objective of this project is a full home service robot capable of navigating to pick up and deliver virtual objects.

<!--more-->

[//]: # (Image References)

[image1]: ./misc_images/HS_Launch1.jpg "./Launch.sh"
[image2]: ./misc_images/HS_Launch2.jpg "./Launch.sh"
[image3]: ./misc_images/HS_Gazebo.jpg "gazebo MyWorld.world"
[image4]: ./misc_images/HS_Test_Slam1.jpg "./test_slam.sh"
[image5]: ./misc_images/HS_Test_Slam2.jpg "./test_slam.sh"
[image6]: ./misc_images/HS_Wall_Follower1.jpg "./wall_follower.sh"
[image7]: ./misc_images/HS_Wall_Follower2.jpg "./wall_follower.sh"
[image8]: ./misc_images/HS_Test_Navigation1.jpg "./test_navigation.sh"
[image9]: ./misc_images/HS_Test_Navigation2.jpg "./test_navigation.sh"
[image10]: ./misc_images/HS_Test_Navigation3.jpg "./test_navigation.sh"
[image11]: ./misc_images/HS_Pick_Objects1.jpg "./pick_objects.sh"
[image12]: ./misc_images/HS_Pick_Objects2.jpg "./pick_objects.sh"
[image13]: ./misc_images/HS_Add_Markers1.jpg "./add_markers.sh"
[image14]: ./misc_images/HS_Add_Markers2.jpg "./add_markers.sh"
[image15]: ./misc_images/HS_Home_Service1.jpg "./home_service.sh"
[image16]: ./misc_images/HS_Home_Service2.jpg "./home_service.sh"


---


#### How to run the program with your own code

Go to Desktop and open a terminal.

For the execution of your own code, we head to the Project Workspace. For this setup, catkin_ws is the name of the active workspace. If your workspace name is different, change the commands accordingly.

If you do not have an active workspace, you can create one. You can launch it by running the following commands first.
```bash
  cd /home/workspace/
  mkdir -p catkin_ws/
```

Clone the required repositories to the `/home/workspace/catkin_ws/` folder
```bash
  cd /home/workspace/catkin_ws/
  git clone https://github.com/Abhaycl/RoboND-Home-Service-Robot-2P5.git src
```

Once copied everything, it's important to give permissions, reading and/or execution to the files necessary for the proper functioning.

Build the project:
```bash
  cd /home/workspace/catkin_ws
  catkin_make
  source devel/setup.bash
```


---

The summary of the files and folders within repo is provided in the table below:

| File/Folder              | Definition                                                                                                   |
| :----------------------- | :----------------------------------------------------------------------------------------------------------- |
| add_markers/*            | Folder that contains all the node that model the object with a marker in rviz.                               |
| pick_objects/*           | Folder that contains all the node that commands your robot to drive to the pickup and drop off zones.        |
| RvizConfig/*             | Folder that contains all the customized rviz configuration files.                                            |
| ShellScripts/*           | Folder that contains all the shell scripts.                                                                  |
| slam_gmapping/*          | Folder that contains all the files that perform SLAM and build a map of the environment with a robot equipped with laser range finder sensors or RGB-D cameras. |
| turtlebot/*              | Folders that contains all the that manually control a robot using keyboard commands, load a preconfigured    |
| turtlebot_apps/*         | rviz workspace. You’ll save a lot of time by launching this file, because it will automatically load the     |
| turtlebot_interactions/* | robot model, trajectories, and map for you and deploy a turtlebot in a gazebo environment by linking the     |
| turtlebot_simulator/*    | world file to it.                                                                                            |
| wall_follower/*          | Folder that contains all the wall_follower node that will autonomously drive your robot around to perform SLAM. |
| World/*                  | Folder that contains all the gazebo world file and the map generated from SLAM.                              |
| misc_images/*            | Folder containing the images of the project.                                                                 |
|                          |                                                                                                              |
| CMakeLists.txt           | Contains the System dependencies that are found with CMake's conventions.                                    |
| README.md                | Contains the project documentation.                                                                          |
| README.pdf               | Contains the project documentation in PDF format.                                                            |

---

**Steps to complete the project:**  

1. Design a simple environment with the Building Editor in Gazebo.
2. Teleoperate your robot and manually test SLAM.
3. Create a wall_follower node that autonomously drives your robot to map your environment.
4. Use the ROS navigation stack and manually commands your robot using the 2D Nav Goal arrow in rviz to move to 2 different desired positions and orientations.
5. Write a pick_objects node that commands your robot to move to the desired pickup and drop off zones. 
6. Write an add_markers node that subscribes to your robot odometry, keeps track of your robot pose, and publishes markers to rviz. 


## [Rubric Points](https://review.udacity.com/#!/rubrics/1442/view)
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
## Execution of the different shellscripts

For run launch.sh open a terminal:

```bash
  cd /home/workspace/catkin_ws/src/ShellScripts
  ./launch.sh
```

![alt text][image1]
![alt text][image2]

---

For create my world open a terminal:

```bash
  cd /home/workspace/catkin_ws/src/World
  gazebo MyWorld.world
```

![alt text][image3]

---

For run test_slam.sh open a terminal:

```bash
  cd /home/workspace/catkin_ws/src/ShellScripts
  ./test_slam.sh
```

![alt text][image4]
![alt text][image5]

---

For run wall_follower.sh open a terminal:

```bash
  cd /home/workspace/catkin_ws/src/ShellScripts
  ./wall_follower.sh
```

![alt text][image6]
![alt text][image7]

---

For run test_navigation.sh open a terminal:

```bash
  cd /home/workspace/catkin_ws/src/ShellScripts
  ./test_navigation.sh
```

![alt text][image8]
![alt text][image9]
![alt text][image10]

---

For run pick_objects.sh open a terminal:

```bash
  cd /home/workspace/catkin_ws/src/ShellScripts
  ./pick_objects.sh
```

![alt text][image11]
![alt text][image12]

---

For run add_markers.sh open a terminal:

```bash
  cd /home/workspace/catkin_ws/src/ShellScripts
  ./add_markers.sh
```

![alt text][image13]
![alt text][image14]

---

For run the project home_service.sh open a terminal:

```bash
  cd /home/workspace/catkin_ws/src/ShellScripts
  ./home_service.sh
```

![alt text][image15]
![alt text][image16]