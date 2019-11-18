# CSIE-Retriever
Final Project of Robotics 2019 Fall Semester.

## Prerequisites

install required ros packages
```
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
```

## Usage

steering in simulation world
```
roslaunch csie-retriever_gazebo csie-retriever.gazebo.launch 
```

steering, navigation and gmapping(SLAM) integration in simulation world
```
roslaunch csie-retriever_navigation csie-retriever_nav_gmapping.launch
```
