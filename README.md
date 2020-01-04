# CSIE-Retriever
Final Project of Robotics 2019 Fall Semester.

## Prerequisites

install required ros packages
```
rosdep install --from-paths src --ignore-src --rosdistro melodic -y
```

## Usage

### Simulation

steering in simulation world
```
roslaunch csie-retriever_gazebo csie-retriever.gazebo.launch 
```

steering, navigation and gmapping(SLAM) integration in simulation world
```
roslaunch csie-retriever_navigation csie-retriever_nav_gmapping.launch
```

### Reality
launch navigation and gmapping(SLAM) integration and Rviz
```
roslaunch csie-retriever_navigation csie-retriever_nav_gmapping.launch dual_scan:=false env:=true
```
workflow controller 
```
rosrun retriever_speech state
```
face detection node
```
rosrun retriever_speech face_detection_auto_id_local.py
```
manual key operator
```
rosrun key_teleop key_teleop.py
```

Finally close key operator to enable guiding mode
