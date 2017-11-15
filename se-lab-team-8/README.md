# QAVI Robot
### Software Engineering Lab - Team 8

## External Dependencies:
### Google Test:
Linux:
    `sudo apt-get install libgtest-dev`

### ROS:
Linux:
    [Install ROS-Indigo] (http://wiki.ros.org/indigo/Installation/Ubuntu)

### OpenCV:
Linux:
    [Install OpenCV v.2.4.8] (http://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html)

### catkin:
Linux:
    `sudo apt-get install cmake python-catkin-pkg python-empy python-nose python-setuptools build-essential`
    `sudo apt-get install ros-indigo-catkin`

### Doxygen:
Linux:
    `sudo apt-get install doxygen`

## Instructions
1. Install dependencies.
2. [Create a workspace for catkin] (http://wiki.ros.org/catkin/Tutorials/create_a_workspace). Remember to do `catkin_make` at least once once you've created your catkin workspace.
3. Clone lab_simulator.git and se-lab-team-8.git in ~/catkin_ws/src/
4. Launch simulation (Each step on a different terminal!). If given a warning that the package with name "lab_simulator" doesn't exist, source your workspace by: `source ~/catkin_ws/devel/setup.bash`
    1. `roscore`
    2. `roslaunch lab_simulator simulator.launch`
5. Run our program on the simulator:
    1. `cd ~/catkin_ws/`
    2. `source devel/setup.bash`
    3. `catkin_make`
    4. `rosrun qavi_robot qavi_robot_node`
6. Run program tests:
    1. `cd ~/catkin_ws/`
    2. `catkin_make run_tests_qavi_robot`
7. Generate Doxygen documentation:
    1. `cd ~/catkin_ws/src/se-lab-team-8/`
    2. `doxygen doxy-config`
    3. In that directory, under the /Documentation/ folder, documentation will be found. To open the html version, the file index.html is found under the /Documentation/html folder.
