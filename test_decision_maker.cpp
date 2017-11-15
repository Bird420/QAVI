/** @file test_circle_detection.cpp
 *  @brief Main test file for circle_detection
 *
 *  Run it by using catkin_make run_tests_qavi_robot
 *
 *  @author Akbar Oripov (aoripov)
 *  @author Inti Mendoza (theElemelon)
 *  @author Qifan Shu (Alprazolam)
 *  @author Vlad Frasineanu (vcfrasineanu)
 *  @bug No known bugs
 */
#include "decision_maker.h"
#include <gtest/gtest.h>

TEST(TestSuite, ImageWithOneCircle)
{
  Mat test = imread(argv[1], 1);
  if (!test.date) {
    return -1 // No image has been fed
  }
  cvtColor(test, test_gray, CV_BGR2Gray);
  EXPECT_EQ(1, (detectSemiCircle(test)).size());
}

TEST(TestSuite,testCase2)
{

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
