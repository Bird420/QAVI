/** @file qavi_robot.cpp
 *  @brief Main function of the robot node
 * 
 *  Currently, this file contains dummy
 *  implementation of robot movement
 *  by processing SensorController data
 *  and moving using MovementController.
 *  In the future should contain only
 *  instantiation of Robot class and 
 *  running this instance.
 *  
 *  @author Akbar Oripov (aoripov)
 *  @author Inti Mendoza (theElemelon)
 *  @author Qifan Shu (Alprazolam)
 *  @author Vlad Frasineanu (vcfrasineanu)
 *  @bug No known bugs
 */

#include "robot.h"

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "qavi_robot"); // initialize node
  ros::NodeHandle n;

  Robot* r = Robot::getRobot(n, 0.3, 0.3);
  r->run();

  return 0;
}

