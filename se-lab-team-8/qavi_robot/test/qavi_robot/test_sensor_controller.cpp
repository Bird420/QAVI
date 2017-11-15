/** @file test_sensor_controller.cpp
 *  @brief Main test file for sensor_controller
 * 
 *  Run it by using catkin_make run_tests_qavi_robot
 *
 *  Tests for each method that it can receive successfully messages of type sensor_msgs::LaserScan via base_scan
 *  
 *  @author Akbar Oripov (aoripov)
 *  @author Inti Mendoza (theElemelon)
 *  @author Qifan Shu (Alprazolam)
 *  @author Vlad Frasineanu (vcfrasineanu)
 *  @bug No known bugs
 */

// included headers
#include "sensor_controller.h"
#include <gtest/gtest.h>

/**
 * @brief Generate pseudo data to pass to sensor
 */
class GenerateData {
  public:

/*------------------*
 * Public Functions *
 *------------------*/
/**
 * @brief Constructor of data generation
 */
    GenerateData() {}

/**
 * @brief Send data to sensor controller for handling
 *
 * This function will create a sensor data that follows
 * the same format as real data the sensor collects
 */
    sensor_msgs::LaserScan sendData(ros::NodeHandle& n)
    {
      ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("base_scan", 1);

      unsigned int num_readings = 720;
      double laser_frequency = 40;
      double ranges[num_readings];
      double intensities[num_readings];

      int count = 0;
      ros::Rate r(1.0);

      //generate some fake data for our laser scan
      for(unsigned int i = 0; i < num_readings; ++i){
        ranges[i] = count;
        intensities[i] = 100 + count;
      }
      ros::Time scan_time = ros::Time::now();

      //populate the LaserScan message
      sensor_msgs::LaserScan scan;
      scan.header.stamp = scan_time;
      scan.header.frame_id = "laser_frame";
      scan.angle_min = -1.57;
      scan.angle_max = 1.57;
      scan.angle_increment = 3.14 / num_readings;
      scan.time_increment = (1 / laser_frequency) / (num_readings);
      scan.range_min = 0;
      scan.range_max = 5.0;

      scan.ranges.resize(num_readings);
      scan.intensities.resize(num_readings);
      for(unsigned int i = 0; i < num_readings; ++i){
        scan.ranges[i] = ranges[i];
        scan.intensities[i] = intensities[i];
      }
      scan_pub.publish(scan);
      ++count;
      r.sleep();
      return scan;
    }
};

/*--------------*
 * Test Methods *
 *--------------*/

/**
 * @brief Test getter methods
 *
 * See if already known values (created in GenerateData class), 
 * when called with getter methods they are returned identically
 */
TEST(TestSensorController, checkGetterMethods)
{
  ros::NodeHandle n;
  SensorController sc(n);

  float* ranges;
  GenerateData data;
  data.sendData(n);

  while (!sc.receivedMessage())
  {
    ros::spinOnce();
  }

  ranges = sc.getRanges();

  float angle_min_test = -1.57;
  float angle_max_test = 1.57;
  float angle_increment_test = 3.14/720;

  EXPECT_TRUE(sc.receivedMessage());
  EXPECT_EQ(angle_min_test, sc.getAngleMin());
  EXPECT_EQ(angle_max_test, sc.getAngleMax());
  EXPECT_EQ(angle_increment_test, sc.getAngleIncrement());
}

/**
 * @brief Initialize GoogleTest
 */
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_sensor_controller"); // initialize node
  ros::NodeHandle n;
  return RUN_ALL_TESTS();
}

