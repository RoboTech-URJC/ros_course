# ROS COURSE

## How to Clone it

You will have to clone this repo using the flag ``--recursive`` because it has other repositories as dependencies.

Clone it into your catkin workspace. Then, compile it. If there are some problems, maybe it will be fixed using ``catkin_make -j1``.

## Packages

### tb3_sim

This packages is located under the *sim* folder. It contains all you need to be able to launch a Gazebo simulation.

You can run it with the following command:

```
$ roslaunch tb3_sim tb3_sim_gui.launch  # GUI
$ roslaunch tb3_sim tb3_sim_nogui.launch    # Not GUI
```

> NOTE: Under the *basics* folder you will find a lot of ROS code examples in Python. Next steps: Add C++ code.