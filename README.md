# Simulation
For all things Simulation

### Building is necessary to make the plugin work.
#### After making a build directory use the following commands to build

```
$ cmake ../
$ make
```
### Post building, you need to set a path for gazebo to know where to check your plugin too.
```
$ export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/path_to_build_directory/build
```
