# Simulation
Repository containing different projects involving any kind of robotic simulation

## Table of contents
* [Project 1](#Project-1)
* [Project 2 (Chase White pixel)](#Project-2) 
* [Project 3 (MCL localization)](#Project-3)
* [Project 4 (RTAB Map)](#Project-4)
* [Project 5 (Home Service)](#Project-5)
## Project 1
##### Spawning a simple three wheeled robot in a simulation environment.
### Build and compile
* Compiling the code in `scripts` directory is necessary to make the plugin work.
* Make a directory names as build, `mkdir build`
* Build directory can be located on the same level as CMakeLists.txt
* After making a build directory, open it `cd build`, then use the following commands to compile the code

```
$ cmake ../
$ make
```
### Post building and compilation, you need to set a path for gazebo to know where to check your plugin too.
```
$ export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/path_to_build_directory/build
```
### Run
* Use the command `gazebo worldSubmit1` to run the simulation

## Project 2
##### Chasing a white colored ball in gazebo simulation by using camera feed.
### Setup and compile
* First the two packages `my_robot` and `ball_chaser` are to be made in src folder of your catkin workspace.
* Then use `catkin_make` in one level before your `src` directory of catkin workspace to build all the executables according to instructions in `CMakeLists.txt` files. 
### Run
* Use the commands `roslaunch my_robot world.launch` and `roslaunch ball_chaser ball_chasr.launch` to run the simulation and move around the ball to test.
### NOTE:
* Play around with torque and velocity values to get smooth turning for different wheel joint coordinates of robot.

## Project 3
##### Observing behaviour of MCL in RViz while tweaking different parameters in AMCL package.

* Particles after initial pose estimation
![Image alt text](thirdProject/Update_0.png?raw=true "Particles after initial pose estimation")


* Particles after 1  update
![Image alt text](thirdProject/Update_1.png?raw=true "Particles after 1  update")


* Particles after 6  updates
![Image alt text](thirdProject/Update_6.png?raw=true "Particles after 6  updates")


* Particles after 30-40  updates
![Image alt text](thirdProject/Update_30-40.png?raw=true "Particles after 30-40  update")

## Project 4
##### Making a map using Real-Time Apearance Based Mapping (RTAB Map). 
##### [Link](https://drive.google.com/drive/folders/10KM1S0ivZaJQQKlwrksGK1FHE_2xCaw6?usp=sharing) to database files(result + exported 3D Map). 

* The yellow dots (in the cube figure) are showing distinct features. (below occupancy grip)
![Image alt text](fourthProject/map_screenshots/databaseViewerShowingFeatures.png?raw=true "databaseViewerShowingFeatures")


* Figures shows the resultant map in RViz in a slanted side view.
![Image alt text](fourthProject/map_screenshots/RvizMapSlantView.png?raw=true "RvizMapSlantView")


* Figures shows the resultant map in RViz in top view.
![Image alt text](fourthProject/map_screenshots/RvizMapTopView.png?raw=true "RvizMapTopView")

## Project 5

## NOTE:- 
* Please change paths from `/home/anvesh/simulation_ws` to your `catkin_workspace` in all scripts and launch files because they were aked to be kept outside packages (example maps folder) could not be given a general path by find command.
* In case you are getting error related to dependencies, try to use `rosdep`
* Do not use original turtlebot packages, the one in repository are modified and must be used only.

* Figures shows the robot going to pick virtual object.
![Image alt text](fifthProject/src/media/going_to_pick.png?raw=true "GoingToPick")

* Figures shows the robot going to drop after picking up.
![Image alt text](fifthProject/src/media/started_moving_to_drop.png?raw=true "started move")
