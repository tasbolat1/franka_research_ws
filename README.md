# franka_research_ws

## PREREQUISITS
1. Install ROS Noetic
2. Python3.7 or above
3. RT kernel based Linux 20.04

## INSTALLATION
Clone the following repo:

```
git clone --recurse-submodules https://github.com/tasbolat1/franka_research_ws.git
```


### Libfranka
Go to the libfranka and build it
```
cd ~/franka_research_ws/src/libfranka
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build .
```
to test libfranka installation, please go to examples

```
cd ~/franka_research_ws/src/libfranka/build/
```

and run 
```
./communication_test <robot_ip>
```

*Troubleshooting for libfranka:*
1. In case libfranka is giving mismatch or unknown error, just build libfranka outside of the franka_research_ws using a method given in documentation.

```
cd ~
git clone --recursive https://github.com/frankaemika/libfranka --branch 0.10.0
cd ~/libfranka
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build .
```
2. If any of the examples in libfranka is asking for sudo permission, then add the current usergroup for rt permission as shown in:

https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel

3. If examples in libfranka do not run due to FCI activation, please set Operations as Execution and activate FCL from webbrowser.

### franka_ros

Pull recursively any additional updates from franka_ros upstream:
```
cd ~/franka_research_ws/src/franka_ros
git submodule update --init --recursive
```

### Building all

```
cd ~/franka_research_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/franka_research_ws/src/libfranka/build
source devel/setup.bash
```
*Note:* catkin_make with libfranka can be used just during first time catkin_make. Afterwards just continue with simple ```catkin_make``` command


## USAGE

Run robot stuff:
```
roslaunch robot_control robot_bringup.launch robot_ip:=172.16.0.2 gripper:=panda
```

Run example of robot control
```
rosrun robot_control move_robot.py
```


## TODO

In a current form, the robot ONLY works with panda_hand. We need to integrate robotiq to it.