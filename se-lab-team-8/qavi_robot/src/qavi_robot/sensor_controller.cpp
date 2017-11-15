/** @file sensor_controller.cpp
 *  @brief SensorController Class methods definition
 * 
 *  This classs cotains methods definition to
 *  receive data from the sensor and convert
 *  this data to the suitable format for use 
 *  in wall/circle/corner detection algorithms
 *  
 *  @author Akbar Oripov (aoripov)
 *  @author Inti Mendoza (theElemelon)
 *  @author Qifan Shu (Alprazolam)
 *  @author Vlad Frasineanu (vcfrasineanu)
 *  @bug No known bugs
 */

// included headers
#include "sensor_controller.h"
#include <math.h>

SensorController::SensorController(ros::NodeHandle &nh) 
{
  nh_ = nh;
  // set up subscriber to "base_scan"
  base_scan_sub_ = nh_.subscribe("base_scan", 1, &SensorController::callback, this);
  matrix_width_ = 1002;
  matrix_height_ = 752;
  scan_ = cv::Mat(matrix_height_, matrix_width_, CV_8U);
  range_max_ = 5.0;
  range_min_ = 0;
  receivedMessage_ = false;
  ROS_INFO("SensorController - Listener initialized");
}

void SensorController::callback(const sensor_msgs::LaserScan::ConstPtr& scan) 
{
  angle_min_ = scan->angle_min;
  angle_max_ = scan->angle_max;
  angle_increment_ = scan->angle_increment;
  ranges_size_ = POINTS_NUM;
  double radian_angle = angle_min_ + M_PI / 2.0;
  int row, col;
  scan_.setTo(cv::Scalar(0)); // initialize matrix with zeros
  for (int i = 0; i < ranges_size_; i++) 
  {
    ranges_[i] = scan->ranges[i];
    // ignore values that are not in the range
    if (ranges_[i] > range_max_ || ranges_[i] < range_min_)
      continue;
    row = std::floor(matrix_height_ / 3 + 1 + sin(radian_angle) * ranges_[i] * 100); // y coordinate
    col = std::floor(matrix_width_ / 2 + cos(radian_angle) * ranges_[i] * 100); // x coordinate
    scan_.at<unsigned char>(row, col) = 255;
    radian_angle += angle_increment_;
  }

  receivedMessage_ = true;
  ROS_INFO("SensorController::callback - Sensor Message received");
}

float* SensorController::getRanges() 
{
  return ranges_;
}

float SensorController::getAngleMin() 
{
  return angle_min_;
}

float SensorController::getAngleMax()
{
  return angle_max_;
}

float SensorController::getAngleIncrement()
{
  return angle_increment_;
}

float SensorController::getRangeMax()
{
  return range_max_;
}

float SensorController::getRangeMin()
{
  return range_min_;
}


cv::Mat* SensorController::getScanImage()
{
  return &scan_;
}

bool SensorController::receivedMessage()
{
  return receivedMessage_;
}

unsigned int SensorController::getRangesSize()
{
  return ranges_size_;
}

void SensorController::setRangeMax(float range_max)
{
  range_max_ = range_max;
}

