# follow_the_gap

Follow the steps below to build, configure, and run your ROS 2 Jazzy project with TurtleBot3 and Gazebo.

---

### 1. Source the ROS 2 environment

```bash
source /opt/ros/jazzy/setup.bash

```
### 2. Build the workspace

```bash
colcon build

```
### 3. Source the workspace setup file
```bash
source install/setup.bash

```
### 4. Set the TurtleBot3 model
```bash
export TURTLEBOT3_MODEL=burger

```

### 5. Launch the TurtleBot3 Gazebo simulation
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

```
This command opens Gazebo, loads the TurtleBot3 Burger model, and spawns it in the default simulation world.

#### Requirements
If you don’t have the necessary packages installed, run the following:
```bash
sudo apt update
sudo apt install ros-jazzy-turtlebot3-gazebo

```

### 6. Run your custom nodes
```bash
ros2 run my_robot_test follow_the_gap
ros2 run my_robot_test scan_filter_180


```



