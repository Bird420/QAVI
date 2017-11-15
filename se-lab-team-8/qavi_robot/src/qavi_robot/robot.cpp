/** @file robot.cpp
 *  @brief Constructor of robot
 *  
 *  @author Akbar Oripov (aoripov)
 *  @author Inti Mendoza (theElemelon)
 *  @author Qifan Shu (Alprazolam)
 *  @author Vlad Frasineanu (vcfrasineanu)
 *  @bug No known bugs
 */

// included headers
#include "robot.h"

Robot* Robot::instance_ = NULL;

Robot::Robot(ros::NodeHandle &nh, float maxSpeed, float wallDistance)
{

  sc_ = new SensorController(nh); // connect to sensor
  mc_ = new MovementController(nh); // connect to drive

  maxSpeed_ = maxSpeed;
  wallDistance_ = wallDistance;
  error_ = 0;

  ROS_INFO_STREAM("QAVI Robot initialized with maximal speed " << maxSpeed_ << " and wall distance" << wallDistance_);
}

Robot* Robot::getRobot(ros::NodeHandle &nh, float maxSpeed, float wallDistance)
{
  if(!instance_)
  {
    instance_ = new Robot(nh, maxSpeed, wallDistance);
  }
  return instance_;
}

void Robot::run()
{
  ros::Rate loop_rate(10);
  ObjectDetector od(sc_);

  bool ok = false;
  bool circle = false;
  int countCircle = 0;
  sc_->setRangeMax(1.0);
  while(ros::ok())
  {
    if (sc_->receivedMessage())
    {
      // move the robot directly to the first wall
      while (!ok)
      {
        while ((sc_->getRanges())[sc_->getRangesSize() / 2] > wallDistance_) 
        {
          mc_->slide(0.1, 0);  
          ros::spinOnce();
          loop_rate.sleep();
        }
        ok = true;
      }
      if (od.isCircleDetected()) 
      {
        countCircle++;
      } else 
      {
        countCircle = 0;
      }
      if (countCircle >= 3)
      {
        circle = true;
        ROS_INFO("Robot::run - Circle Detected");
      }
      if (!circle) 
      {
        float minIndex = od.getClosestWallIndex();

        decide(minIndex);
        mc_->slide(command_.first,command_.second);
      }
      else
      {
        mc_->slide(maxSpeed_, od.getTurnAngle());
      }
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }
}

bool Robot::decide(unsigned int minIndex)
{
  int size = sc_->getRangesSize();
  
  int angleMin = (minIndex-sc_->getRangesSize() / 2)*sc_->getAngleIncrement();
  double distMin;
  distMin = (sc_->getRanges())[minIndex];
  double distFront = (sc_->getRanges())[size / 2];
  double diffE = (distMin - wallDistance_) - error_;
  error_ = distMin - wallDistance_;

  command_.second = (P*error_ + D*diffE) + angleCoef * (angleMin - PI / 2);    //fix angular speed

  // fix linear speed
  if (distFront < wallDistance_)
  {
    command_.first = 0;
  }
  else if (distFront < wallDistance_ * 2)
  {
    command_.first = 0.5 * maxSpeed_;
  }
  else if (fabs(angleMin) > 1.75)
  {
    command_.first = 0.4 * maxSpeed_;
  } 
  else 
  {
    command_.first = maxSpeed_;
  }
  return true;
}

