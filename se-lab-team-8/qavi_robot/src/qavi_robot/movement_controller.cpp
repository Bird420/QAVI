/** @file movement_controller.cpp
 *  @brief MovementController Class with methods to move robot
 * 
 *  This classs cotains methods to move robot
 *  The movement is described by moving forward
 *  and turnig left or right
 *  
 *  @author Akbar Oripov (aoripov)
 *  @author Inti Mendoza (theElemelon)
 *  @author Qifan Shu (Alprazolam)
 *  @author Vlad Frasineanu (vcfrasineanu)
 *  @bug No known bugs
 */

// included headers
#include "movement_controller.h"

MovementController::MovementController(ros::NodeHandle &nh)
{
  nh_ = nh;
  //set up the publisher for the cmd_vel topic
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ROS_INFO("MovementController - Publisher initialized");
}

void MovementController::moveForward(float speed)
{
  //we will be sending commands of type "twist"
  geometry_msgs::Twist base_cmd;
  base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;

  base_cmd.linear.x = speed;
  cmd_vel_pub_.publish(base_cmd);
  ROS_INFO_STREAM("MovementController::moveForward - command published using speed: " 
	<< speed);
}

void MovementController::turn(float angle)
{
  //we will be sending commands of type "twist"
  geometry_msgs::Twist base_cmd;
  base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;

  base_cmd.angular.z = angle;
  cmd_vel_pub_.publish(base_cmd);
  ROS_INFO_STREAM("MovementController::turn - command published using speed: " << angle);
}

void MovementController::slide(float speed, float angle)
{
  //we will be sending commands of type "twist"
  geometry_msgs::Twist base_cmd;
  base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;

  base_cmd.linear.x = speed;
  base_cmd.angular.z = angle;
  cmd_vel_pub_.publish(base_cmd);
  ROS_INFO_STREAM("MovementController::slide - command published using linear speed: " 
	<< speed << " and angular speed: " << angle);
}

void MovementController::stop()
{
  //we will be sending commands of type "twist"
  geometry_msgs::Twist base_cmd;
  base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;

  cmd_vel_pub_.publish(base_cmd);
  ROS_INFO("MovementController::stop - command published");
}

