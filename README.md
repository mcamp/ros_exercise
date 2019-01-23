# ros_exercise

An exercise of ROS using a simulated [husky](http://wiki.ros.org/Robots/Husky) in ROS Indigo


## Preparations

First, we need to install the husky libraries:

```
sudo apt-get install ros-indigo-husky-simulator
sudo apt-get install ros-indigo-husky-navigation
```

## Installing and Running

First, we need to install the package in a catkin space. 
You just need to clone this repository in your catkin space.

Then, you launch the file:

```
roslaunch ros_exercise husky.launch
```

This will spawn the Gazebo simulation with the husky and the RViz visualization.

## Follow using RViz

To make the robot move, we need to use the **2D Pose Estimate** to add a new pose to follow.
An arrow will appear to show where the robot will move.
You can keep adding new poses and the robot should follow them in order.

We use the **2D Pose Estimate** for simplicity. 
We can not use the **2D Nav Goal** because it is used for the navigation stack to move the robot.
