/** @file movement_controller.h
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

#ifndef MOVEMENT_CONTROLLER_H
#define MOVEMENT_CONTROLLER_H

// included headers
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>

/** 
 * @brief This class holds the movement commands that
 * are passed to the robot so that it may perform
 * desired displacement.
 */

class MovementController
{

/*-----------------------------*
 * Private Member Declarations *
 *-----------------------------*/

private:
/**
 * @brief The node handle we'll be using
 */
  ros::NodeHandle nh_;
/**
 * @brief We will be publishing to the "cmd_vel" topic to issue commands
 */
  ros::Publisher cmd_vel_pub_;

public:

/*------------------*
 * Public Functions *
 *------------------*/

/**
 * @brief ROS node intialization
 * 
 * How to initialize a movement controller:
 * init the ROS node
 * ros::init(argc, argv, "movement_controller");
 * ros::NodeHandle nh;
 * MovementController moveControl(nh);
 * moveControl.moveForward(speed) etc.
 * 
 * @param  nh NodeHandle
 */
  MovementController(ros::NodeHandle &nh);

/**
 * @brief Move robot in the field
 * 
 * This method sends data via /cmd_vel topic
 * to the Drive node to move forward with
 * defined speed.
 * 
 * @param  speed Speed to move forward
 * @return Void
 */
  void moveForward(float speed);

/**
 * @brief Rotate robot in the field
 * 
 * This method sends data via /cmd_vel topic
 * to rotate robot counterclockwise (left)
 * and clockwise (right)
 *  
 * @param  angle Speed to move forward
 * @return Void
 */
  void turn(float angle);

/**
 * @brief Slide robot in the field
 * 
 * This method sends data via /cmd_vel topic
 * to slide robot counterclockwise (left)
 * and clockwise (right)
 *  
 * @param  speed Speed to move forward
 * @param  angle Speed to move counterclockwise or clockwise
 * @return Void
 */
  void slide(float speed, float angle);

/**
 * @brief Stop robot in the field
 * 
 * This method sends data via /cmd_vel topic
 * to decrease both speeds of the robot to 0
 *  
 * @return Void
 */
  void stop();
};

#endif /* MOVEMENT_CONTROLLER_H */

