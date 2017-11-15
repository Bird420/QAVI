/** @file robot.h
 *  @brief Robot main class which operates sensor data 
 *         and sends data to the movement controller
 *
 *  This is a singleton class for a robot object
 *
 *  @author Akbar Oripov (aoripov)
 *  @author Inti Mendoza (theElemelon)
 *  @author Qifan Shu (Alprazolam)
 *  @author Vlad Frasineanu (vcfrasineanu)
 *  @bug No known bugs
 */

// included headers
#include "sensor_controller.h"
#include "movement_controller.h"
#include "object_detector.h"

 /*---------*
 * Defines *
 *---------*/
/**
 * @brief Math constant PI
 */
const double PI = 3.1415;

/**
 * @brief Proportional constant for controller
 */
const unsigned int P = 10;

/**
 * @brief Derivative constant for controller
 */
const unsigned int D = 5;

/**
 * @brief Proportional constant for angle controller
 */
const unsigned int angleCoef = 1;

/**
 * @brief Class to command correctly our robot
 *
 * This class instantiates our robot and proper
 * controllers to obtain data from it to decide
 * proper course of action to then command to
 * robot
 */

class Robot
{
private:

/*-----------------------------*
 * Private Member Declarations *
 *-----------------------------*/

/**
 * @brief Create instance of Robot as pointer
 */
  static Robot* instance_;  

/**
 * @brief Create instance of SensorController as pointer
 *
 * From which we'll get sensor data from robot
 */
  SensorController* sc_;

/**
 * @brief Create instance of Movement Controller as pointer
 *
 * Which we will use to command our robot's movements
 */
  MovementController* mc_;
  
/**
 * @brief Create pair to store values required to be in pairs
 *
 * Will be used to store speed and angle of turn our robot must
 * follow
 */
  //pair<linear speed, angular speed>
  std::pair<float,float> command_; 

/**
 * @brief Margin of error acceptable
 *
 * Difference between desired distance from the wall and actual distance.
 */
  float error_;

/**
 * @brief Create variable to store maximum speed allowed for robot
 */
  float maxSpeed_;

/**
 * @brief Create variable to store distance from robot to wall fed
 */
  float wallDistance_;

/**
 * @brief private constructor
 *
 * This is the constructor for robot class
 * which is not intended to be used by
 * functions other than getRobot()
 *
 */
  Robot(ros::NodeHandle &nh, float maxSpeed, float wallDistance);

public:

/*------------------*
 * Public Functions *
 *------------------*/

/**
 * @brief function to get singleton object
 *
 * Will return the robot instance pointer
 * and will create one if it does not exist.
 * Takes as parameter the maximal speed that we want
 * our robot to have and its maximal distance that we want
 * to keep from the walls during the moving of the robot.
 *
 * @return Robot
 */
  static Robot* getRobot(ros::NodeHandle &nh, float maxSpeed, float wallDistance);

/**
  * @brief the main loop
  *
  * This is the main loop of this robot
  * please run the robot from here.
  * First takes the minIndex from the object detector
  * and sends it to decide method.
  * Then, takes the updated command and sends it to the 
  * movement controller
  *
  * @return void
  */
  void run();


/**
 * @brief function to make decision
 *
 * Updates the command taken by the movement controller
 * by computing angular and linear speed.
 *
 * @param minIndex returned by the object detector
 * @return whether the function terminates normally
 */
  bool decide(unsigned int minIndex);
};

