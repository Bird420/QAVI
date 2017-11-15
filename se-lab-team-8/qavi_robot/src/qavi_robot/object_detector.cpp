/** @file object_detector.cpp
 *  @brief ObjectDetector  class definition
 *  @author Akbar Oripov (aoripov)
 *  @author Inti Mendoza (theElemelon)
 *  @author Qifan Shu (Alprazolam)
 *  @author Vlad Frasineanu (vcfrasineanu)
 *  @bug No known bugs
 */

#include "object_detector.h"

ObjectDetector::ObjectDetector(SensorController* sc)
{
  sc_ = sc;
  turn_angle_ = 0;
  ROS_INFO("ObjectDetector initialized");
}

float ObjectDetector::getTurnAngle()
{
  return turn_angle_;
}

bool ObjectDetector::isCircleDetected()
{
  cv::Mat src_gray;
  sc_->getScanImage()->copyTo(src_gray);
  /// Reduce the noise so we avoid false circle detection
  cv::GaussianBlur( src_gray, src_gray, cv::Size(3, 3), 2, 2 );
  std::vector<cv::Vec3f> circles;
  // convert to binary image
  threshold(src_gray, src_gray, 0, 255, cv::THRESH_BINARY);
  // detect edged to make circle more clear
  cv::Canny(src_gray, src_gray, 10, 255, 5, true);
  /// Apply the Hough Transform to find the circles
  cv::HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, 1000, 100, 14.8, 15, 30);
  if(!circles.empty() && circles.size() == 1)
  {
    float dx = 500 - circles[0][0];
    float dy = 250 - circles[0][1];
    if(dy < 0) //meaning the center is actually behind the robot
    {
      turn_angle_ = M_PI - atan(dx / dy);
    }
    else
    {
      turn_angle_ = -atan(dx/dy);
    }
    ROS_INFO_STREAM("ObjectDetector::isCircleDetected - Circle detected at angle " << turn_angle_);
    return true;
  }
  turn_angle_ = 0;
  return false;
}

unsigned int ObjectDetector::getClosestWallIndex()
{
  unsigned int toIndex = sc_->getRangesSize() - 1; // initialize minimum to left most range value
  unsigned int fromIndex = sc_->getRangesSize() / 2; // consider only ranges that are left to the robot
  unsigned int minIndex = fromIndex;
  float* ranges = sc_->getRanges();
  for (int i = fromIndex; i < toIndex; i++)
  {
    if (ranges[i] < ranges[minIndex])
    {
      minIndex = i;
    }
  }
  return minIndex;
}

