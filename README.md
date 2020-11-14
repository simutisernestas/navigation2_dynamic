# navigation2_dynamic
Navigation2's dynamic obstacle detection, tracking, and processing pipelines.

## Requirements
- Ubuntu 20.04
- ROS foxy
- python 3.8

## Installation
1. Install detectron2 following the [instruction](https://github.com/facebookresearch/detectron2/blob/master/INSTALL.md).
2. Clone the repo to the workspace & build & source
```
cd path/to/ros_ws/src
git clone https://github.com/tony23545/navigation2_dynamic.git
colcon build
source install/setup.bash
```

## Run
1. Config the topic name for the pointcloud data from your camera [here](https://github.com/tony23545/navigation2_dynamic/blob/master/detectron2_detector/config/detectron2.yaml) and launch it. 
2. launch the detector node
```
ros2 launch detectron2_detector detectron.launch.py
```
3. launch the tracker node
```
ros2 launch kf_hungarian_tracker kf_hungarian.launch.py
```
4. now the tracking results are published on the topic `tracking` and rviz marker visualization on `marker`.
