# RobotechVirtualChallenge

## Launching the Robotech virtual challenge world

### Clone the repo in your workspace

### Configuring your system
```console
  source ~/[robotech_ws]/devel/setup.bash
  export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:[robotech_ws]/src/RobotechVirtualChallenge/robots/worlds/models
```
### Launching Gazebo
```console
  roslaunch robots sim_gui.launch
```
Gazebo opens with the first iteration of the maze and a kobuki robot ready to test the maze.
