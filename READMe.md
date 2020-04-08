# RRT for Path Planning in ROS

## What it does
It takes in an Occupancy Grid map, initial pose and a goal and gives out a path using RRT.

## How to run it?
Place it in your catkin workspace `src` folder and build it using:
```shell script
catkin_make
```
OR
```shell script
catkin build
```
You can run the launch file given in the `launch` folder using:
```shell script
roslaunch rrt_planner rrt_planner.launch
```

Once it receives the map it will ask you to give the goal and the the initial pose through RViz. 
Or you can provide it through the topics `/initialpose` and `/move_base_simple/goal` respectively.

## Tuning
You can tune the algorithm to your needs by using the `config.yaml` file present in the `cfg` folder. 
The visualization can be turned off from there too.
