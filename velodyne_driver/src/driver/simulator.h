/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver interface for the Velodyne 3D LIDARs
 */

#ifndef _VELODYNE_SIMULATOR_H_
#define _VELODYNE_SIMULATOR_H_

#include <string>
#include <ros/ros.h>

#include <velodyne_driver/output.h>
#include <velodyne_msgs/VelodyneScan.h>

namespace velodyne_driver
{

class VelodyneSimulator
{
public:

  VelodyneSimulator(ros::NodeHandle node,
                 ros::NodeHandle private_nh);
  ~VelodyneSimulator() {}

  void scanCallback(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg);

private:

  // configuration parameters
  std::string model;               ///< device model name
  int    npackets;                 ///< number of packets to collect
  double rpm;                      ///< device rotation rate (RPMs)
  double time_offset;              ///< time in seconds added to each velodyne time stamp


  boost::shared_ptr<Output> output_;
  ros::Subscriber input_;
};

} // namespace velodyne_simulator

#endif // _VELODYNE_SIMULATOR_H_
