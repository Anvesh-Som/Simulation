# Simulation
Repository containing different projects involving any kind of robotic simulation

## Table of contents
* [Project 1](#Project-1)
## Project 1
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
