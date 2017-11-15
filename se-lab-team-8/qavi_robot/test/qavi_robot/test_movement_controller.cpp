/** @file test_movement_controller.cpp
 *  @brief Main test file for movement_controller
 *
 *  Tests for each method that it can send successfully messages of type Twist via cmd_vel
 *   
 *  @author Akbar Oripov (aoripov)
 *  @author Inti Mendoza (theElemelon)
 *  @author Qifan Shu (Alprazolam)
 *  @author Vlad Frasineanu (vcfrasineanu)
 *  @bug No known bugs
 */

// included headers
#include "movement_controller.h"
#include <gtest/gtest.h>

/**
 * @brief Initializes a Subscriber to send callback message
 */

class TestSubscriber {

/*------------------*
 * Public Functions *
 *------------------*/
  public:

/**
 * @brief Constructs subscriber and sets variable to check if message has been received
 */
    TestSubscriber() : receivedMessage(false) 
    {
    }

/*
 * @brief Callback function to receive message
 */
    void callback(const geometry_msgs::Twist &newMessage)
    {
      receivedMessage = true;
      message = newMessage;
    }

/*----------------------------*
 * Public Member Declarations *
 *----------------------------*/

/**
 * @brief Create variable to check if message has been received
 */
    bool receivedMessage;

/**
 * @brief Create message to store any messages received 
 */
    geometry_msgs::Twist message;
};

/*--------------*
 * Test Methods *
 *--------------*/

/**
 * @brief Test moveForward Method
 * 
 * Checks if when given the command to move forward,
 * the robot actually moves forward as commanded
 */
TEST(TestMovementController, moveForwardMethod)
{
  ros::NodeHandle n;
  
  MovementController mc(n); 

  TestSubscriber subscriber;
  ros::Subscriber sub = n.subscribe("cmd_vel", 1, &TestSubscriber::callback, &subscriber);

  ASSERT_EQ(1, sub.getNumPublishers());

  mc.moveForward(0.5);
  while (!subscriber.receivedMessage)
  {
    ros::spinOnce();
  }

  EXPECT_TRUE(subscriber.receivedMessage);

  float x = 0.5;
  ASSERT_EQ(x, subscriber.message.linear.x);

  ASSERT_EQ(0, subscriber.message.linear.y);
  ASSERT_EQ(0, subscriber.message.linear.z);

  ASSERT_EQ(0, subscriber.message.angular.x);
  ASSERT_EQ(0, subscriber.message.angular.y);
  ASSERT_EQ(0, subscriber.message.angular.z); 
}

/**
 * @brief Test turn Method
 * 
 * Checks if when given the command to turn,
 * the robot actually turns as commanded
 */
TEST(TestMovementController, turnMethod)
{
  ros::NodeHandle n;
  
  MovementController mc(n); 

  TestSubscriber subscriber;
  ros::Subscriber sub = n.subscribe("cmd_vel", 1, &TestSubscriber::callback, &subscriber);

  ASSERT_EQ(1, sub.getNumPublishers());

  mc.turn(1.57);
  while (!subscriber.receivedMessage)
  {
    ros::spinOnce();
  }

  EXPECT_TRUE(subscriber.receivedMessage);

  float x = 1.57;
  ASSERT_EQ(x, subscriber.message.angular.z); 

  ASSERT_EQ(0, subscriber.message.linear.x);
  ASSERT_EQ(0, subscriber.message.linear.y);
  ASSERT_EQ(0, subscriber.message.linear.z);

  ASSERT_EQ(0, subscriber.message.angular.x);
  ASSERT_EQ(0, subscriber.message.angular.y); 
}

/**
 * @brief Test slide Method
 * 
 * Checks if when given the command to slide,
 * the robot actually slides as commanded
 */
TEST(TestMovementController, slideMethod)
{
  ros::NodeHandle n;
  
  MovementController mc(n); 

  TestSubscriber subscriber;
  ros::Subscriber sub = n.subscribe("cmd_vel", 1, &TestSubscriber::callback, &subscriber);

  ASSERT_EQ(1, sub.getNumPublishers());

  mc.slide(-0.5, 1.57);
  while (!subscriber.receivedMessage)
  {
    ros::spinOnce();
  }

  EXPECT_TRUE(subscriber.receivedMessage);

  float x = -0.5; 
  float y = 1.57;
  ASSERT_EQ(x, subscriber.message.linear.x);
  ASSERT_EQ(y, subscriber.message.angular.z);

  ASSERT_EQ(0, subscriber.message.linear.y);
  ASSERT_EQ(0, subscriber.message.linear.z);

  ASSERT_EQ(0, subscriber.message.angular.x);
  ASSERT_EQ(0, subscriber.message.angular.y); 
}

TEST(TestMovementController, stopMethod)
{
  ros::NodeHandle n;
  
  MovementController mc(n); 

  TestSubscriber subscriber;
  ros::Subscriber sub = n.subscribe("cmd_vel", 1, &TestSubscriber::callback, &subscriber);

  ASSERT_EQ(1, sub.getNumPublishers());

  mc.stop();
  while (!subscriber.receivedMessage)
  {
    ros::spinOnce();
  }

  EXPECT_TRUE(subscriber.receivedMessage);

  ASSERT_EQ(0, subscriber.message.linear.x);
  ASSERT_EQ(0, subscriber.message.linear.y);
  ASSERT_EQ(0, subscriber.message.linear.z);

  ASSERT_EQ(0, subscriber.message.angular.x);
  ASSERT_EQ(0, subscriber.message.angular.y); 
  ASSERT_EQ(0, subscriber.message.angular.z);
}

/**
 * @brief Initialize GoogleTest
 */
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_movement_controller"); // initialize node
  ros::NodeHandle n;
  return RUN_ALL_TESTS();
}

