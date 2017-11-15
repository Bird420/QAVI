/** @file algorithm.cpp
 *  @brief Algorithm class declaration
 * 
 *  This file cotains declaration of
 *  Algorithm class. This class contains
 *  different types of shaped detection
 *  algoritms (wall/corner/semi-circle)
 *  
 *  @author Akbar Oripov (aoripov)
 *  @author Inti Mendoza (theElemelon)
 *  @author Qifan Shu (Alprazolam)
 *  @author Vlad Frasineanu (vcfrasineanu)
 *  @bug No known bugs
 */

#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

// included headers
#include <vector>
#include <cmath>
#include <ros/console.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "sensor_controller.h"

/**
 * @brief Detect circle and wall
 *
 * This class aims to detect objects that
 * are in our robot's arch of sight 
 */

class ObjectDetector
{
public:

/*------------------*
 * Public Functions *
 *------------------*/

  /** 
   * @brief Constructor for ObjectDetector
   * 
   * @param sc_ Pointer to SensorController instance
   */
  ObjectDetector(SensorController *sc);

  /**
   * @brief Get turn angle
   *  
   * Get angle between the sensor
   * and the center of the circle
   *  
   * @return turn angle
   */
  float getTurnAngle();

  /**
   * @brief Detect semi circle in image
   *
   * @return True if circle is detected, false otherwise
   */
  bool isCircleDetected();

  /**
   * @brief Compute distance to closest wall from the left
   * 
   * @return index of ranges from the scan to the closest wall
   */
  unsigned int getClosestWallIndex();

private:

/*-----------------------------*
 * Private Member Declarations *
 *-----------------------------*/

/**
 * @brief Save desired angle
 *
 * This variable will be sent to other functions to ensure
 * proper turning angle of the robot
 */
  float turn_angle_;

/**
 * @brief Instantiate SensorController as Pointer
 *
 * This instance will feed us sensor data from robot
 */
  SensorController *sc_;
};

#endif /* OBJECT_DETECTOR_H */

