# install

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt-get install ros-kinetic-p2os-driver ros-kinetic-p2os-teleop ros-kinetic-p2os-launch ros-kinetic-p2os-urdf
```

# init

```
sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

# test

```
roscore
```

# build 


```
mkdir -p ~/my_ws/src
cd ~/my_ws/src
git clone https://github.com/BennyTW/CSIE-Retriever.git
cd ~/my_ws
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
catkin_make
source devel/setup.bash
```

# test

- steering in simulation world

```
roslaunch csie-retriever_gazebo csie-retriever.gazebo.launch 
```

- steering, navigation and gmapping(SLAM) integration in simulation world

```
roslaunch csie-retriever_navigation csie-retriever_nav_gmapping.launch
```