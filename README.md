Cardiff Autonomous Racing FS-AI 2025: Perception Stack

ROS2-based perception pipeline for cone detection and SLAM in the EUFS sim

## Packages:

- `cone_detector/`: Colour based (HSV) cone detection publishing cone position + colour.
- `slam_example/`: Launch files + config to run ORB-SLAM3 using ZED2 camera simulation.
- `eufs_sim/`: Sim environment (Boogiemanc fork but plugins from official EUFS repo).
- `ackermann_msgs/`, `eufs_msgs/`: Dependencies for EUFS sim

## Dependencies

Install these system packages first:

```bash
sudo apt install libeigen3-dev libpangolin-dev libopencv-dev
```

You’ll also need to clone and build:

ORB-SLAM3

Pangolin

Clone them into your workspace like this:

```bash
cd ~/your_ws
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
git clone https://github.com/stevenlovegrove/Pangolin.git
```

Then follow this tutorial: https://medium.com/@antonioconsiglio/integrating-orb-slam3-with-ros2-humble-on-raspberry-pi-5-a-step-by-step-guide-78e7b911c361 to compile and integrate with ROS2.

## Running the System

Build the workspace:
```bash
colcon build --symlink-install
source install/setup.bash
```
In each new terminal: 
```
source your_ws/install/setup.bash
```
Launch simulation:
```
export EUFS_MASTER=true
ros2 launch eufs_tracks rectangle.launch
```
Launch cone detection node:
```
ros2 run cone_detector cone_detector_node
```
Launch SLAM:
```
ros2 launch slam_example slam_example.launch.py
```
