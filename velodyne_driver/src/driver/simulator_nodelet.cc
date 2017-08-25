/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver nodelet for the Velodyne 3D LIDARs
 */

#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "simulator.h"

namespace velodyne_driver
{

class SimulatorNodelet: public nodelet::Nodelet
{
public:

  SimulatorNodelet()
  {}

  ~SimulatorNodelet()
  {
  }

private:

  virtual void onInit(void);

  boost::shared_ptr<VelodyneSimulator> simulator_; ///< simulator implementation class
};

void SimulatorNodelet::onInit()
{
  NODELET_INFO_STREAM ("Initializing velodyne spoofing nodelet!");
  // start the driver
  simulator_.reset(new VelodyneSimulator(getNodeHandle(), getPrivateNodeHandle()));

}


} // namespace velodyne_driver

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_driver, SimulatorNodelet,
                        velodyne_driver::SimulatorNodelet, nodelet::Nodelet);
