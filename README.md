# Go Chase It

This repository contains the **Go Chase It** project from the Udacity Robotics Software Engineer Nanodegree.

## Project Overview

In this project, a differential-drive robot is spawned in Gazebo with a camera and lidar. A C++ perception node analyzes camera images to detect a white ball and commands the robot to chase it through a ROS service.

https://github.com/user-attachments/assets/9db8994e-2ed0-4924-9713-d940e73f2f37

## Recent Updates

- Added perimeter boundary walls in the active world to keep the simulation fully enclosed.
- Lowered white-ball detection threshold from 255 to 200 for more reliable detection under varied lighting.
- Improved `process_image` control logic to use centroid-based steering and smoother velocity commands while approaching the ball.

## Implemented Packages

The workspace includes these ROS packages under `catkin_ws/src`:

- `my_robot`
- `ball_chaser`

### 1) my_robot package

The `my_robot` package contains the robot model and simulation world setup.

Completed work:
- Built a differential-drive robot using URDF/Xacro.
- Added two sensors: camera and lidar.
- Added Gazebo plugins for:
  - Differential drive control
  - Lidar sensor
  - Camera sensor
- Added launch files to load the robot description and start Gazebo world simulation.

### 2) ball_chaser package

The `ball_chaser` package contains service definitions and C++ nodes for chasing behavior.

Completed work:
- Created `DriveToTarget.srv` service.
- Implemented `drive_bot` node:
  - Provides `/ball_chaser/command_robot` service.
  - Receives requested `linear_x` and `angular_z` values.
  - Publishes velocity commands to `/cmd_vel`.
  - Returns feedback with commanded values.
- Implemented `process_image` node:
  - Subscribes to `/camera/rgb/image_raw`.
  - Detects white pixels corresponding to the white ball.
  - Determines whether the ball is left, center, or right in the image.
  - Calls the service client to steer robot toward the ball.
- Added `ball_chaser.launch` to run both nodes:
  - `drive_bot`
  - `process_image`

## Directory Layout

```text
go-chase-it/
  catkin_ws/
    src/
      my_robot/
      ball_chaser/
```

## Build Instructions

From the project root:

```bash
cd catkin_ws
catkin_make
```

Source the workspace:

```bash
source devel/setup.bash
```

## Run Instructions

Open terminal 1 and launch the world and robot:

```bash
cd catkin_ws
source devel/setup.bash
roslaunch my_robot world.launch
```

Open terminal 2 and start ball-chasing nodes:

```bash
cd catkin_ws
source devel/setup.bash
roslaunch ball_chaser ball_chaser.launch
```

## Expected Behavior

- The robot camera detects the white ball.
- The `process_image` node detects bright ball pixels (threshold 200) and computes the ball centroid in the image.
- The node requests `/ball_chaser/command_robot` service.
- The `drive_bot` node publishes smooth linear and angular commands to `/cmd_vel`.
- The robot steers proportionally toward the ball, slows while turning, and stops if the ball is not visible.
