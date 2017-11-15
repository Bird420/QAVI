# QAVI Robot
### Software Engineering Lab - Team 8

## External Dependencies:
### Google Test:
Linux:
    sudo apt-get install libgtest-dev

### ROS:
Linux:
    [Install ROS-Indigo] (http://wiki.ros.org/indigo/Installation/Ubuntu)

### OpenCV:
Linux:
    [Install OpenCV v.2.4.8] (http://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html)

### catkin:
Linux:
    sudo apt-get install cmake python-catkin-pkg python-empy python-nose python-setuptools build-essential
    sudo apt-get install ros-indigo-catkin

## Instructions
1. Install dependencies.
2. [Create a workspace for catkin] (http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
3. Clone lab_simulator.git and se-lab-team-8.git in ~/catkin_ws/src/
4. Launch simulation
    * Each step on a different terminal!
    1. roscore
    2. roslaunch lab_simulator simulator.launch
    3. rviz -d `rospack find lab_simulator`/rviz/basic.rviz
    * If given a warning that the package with name "lab_simulator" doesn't exist, source your workspace by: source ~/catkin_ws/devel/setup.bash
4. Run our program on the simulator
    * On a new terminal.
    1. cd ~/catkin_ws/
    2. catkin_make
    3. rosrun qavi_robot qavi_robot_node
5. Run program tests
    1. cd ~/catkin_ws/
    2. catkin_make run_tests_qavi_robot

