/** @file sensor_controller.h
 *  @brief SensorController Class with methods to receive data from sensor
 * 
 *  This classs cotains methods to 
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

#ifndef SENSOR_CONTROLLER_H
#define SENSOR_CONTROLLER_H

// included headers
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/opencv.hpp>

/*---------*
 * Defines *
 *---------*/
/**
 * @brief Number of distances our sensor detects
 */

const unsigned int POINTS_NUM = 720;
/**
 * @brief Get data from our robot's sensor
 */
class SensorController
{
private:

/*-----------------------------*
 * Private Member Declarations *
 *-----------------------------*/

/**
 * @brief The node handle we'll be using
 */
  ros::NodeHandle nh_;

/**
 * @brief We will subscribe to the "base_scan" topic to get scan data
 */  
  ros::Subscriber base_scan_sub_;

/**
 * @brief Create variable to store sensor's minimum angle
 */
  float angle_min_;

/**
 * @brief Create variable to store sensor's maximum angle
 */
  float angle_max_;

/**
 * @brief Create variable to store sensor's difference between adjacent angles
 */
  float angle_increment_;

/**
 * @brief Create variable to store sensor's maximum range
 */
  float range_max_;

/**
 * @brief Create variable to store sensor's minimum range
 */
  float range_min_;

/**
 * @brief Create variable to store size of sensor's range
 */
  unsigned int ranges_size_;

/**
 * @brief Create array to store sensor's incoming data
 */
  float ranges_[POINTS_NUM];

/**
 * @brief Create variable to store the width of a matrix to create image
 */
  float matrix_width_;

/**
 * @brief Create variable to stroe the height of a matrix to create image
 */
  float matrix_height_;

/**
 * @brief Create matrix to draw image from  sensor's data
 */
  cv::Mat scan_;

/**
 * @brief Check if the sensor controller received the data
 */
  bool receivedMessage_;

public:

/*------------------*
 * Public Functions *
 *------------------*/

/**
 * @brief ROS node intialization
 * 
 * Constructor for SensorController
 * 
 * @param  nh NodeHandle
 */
  SensorController(ros::NodeHandle &nh);

/**
 * @brief Callback to get LaserScan data
 * 
 * This method is responsible for asynchnonous
 * receiveing LaserScan data from the
 * base_scan topic
 * 
 * @param scan LaserScan data
 */
  void callback(const sensor_msgs::LaserScan::ConstPtr& scan);
  
/**
 * @brief Getter for ranges
 * 
 * @return Array of ranges as a pointer to float
 */
  float* getRanges();

/**
 * @brief Getter for minimum angle of scan
 * 
 * @return Minimum angle of scan
 */
  float getAngleMin();

/**
 * @brief Getter for maximum angle of scan
 * 
 * @return Maximum angle of scan
 */
  float getAngleMax();

/**
 * @brief Getter angle between scans
 * 
 * @return Angle between scans in radians
 */
  float getAngleIncrement();

/**
 * @brief Getter for maximum range of the sensor
 * 
 * @return Maximum range of the sensor
 */
  float getRangeMax();

/**
 * @brief Getter for minimum range of the sensor
 * 
 * @return Minimum range of the sensor
 */
  float getRangeMin();

/**
 * @brief Function returns point cloud image
 *
 * This method returns point clodu image as
 * OpenCV Mat. Image is created by converting
 * polar coordinates from the sensor to the 
 * cartesian ones and mapping this coordinates
 * to the Mat
 * 
 * @return OpenCV Mat<unsigned char> pointer
 */
  cv::Mat* getScanImage();


/**
 * @brief Getter for size of ranges
 * 
 * @return Size of ranges
 */
  unsigned int getRangesSize();

/**
 * @brief Check if the sensor controller recieved a message
 * 
 * @return True or False
 */
  bool receivedMessage();

  /**
 * @brief Setter for range_max
 * 
 * @param range_max Maximum range
 */
  void setRangeMax(float range_max);
};

#endif /* SENSOR_CONTROLLER_H */

