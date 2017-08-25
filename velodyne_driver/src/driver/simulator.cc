/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the Velodyne 3D LIDARs
 */

#include <string>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "simulator.h"

namespace velodyne_driver
{
VelodyneSimulator::VelodyneSimulator(ros::NodeHandle node,
                               ros::NodeHandle private_nh)
{
  int udp_port;
  private_nh.param("port", udp_port, (int) DATA_PORT_NUMBER);
  std::string dest_ip_addr;
  private_nh.param("dest_ip", dest_ip_addr, std::string(""));
  output_.reset(new velodyne_driver::OutputSocket(private_nh, dest_ip_addr, udp_port));

  // raw packet input topic
  input_ = node.subscribe<velodyne_msgs::VelodyneScan>("velodyne_packets", 10, &VelodyneSimulator::scanCallback, (VelodyneSimulator *) this);
}


void VelodyneSimulator::scanCallback(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
{
  // Get number of packets in scan
  int num_packets = scanMsg->packets.size();

  //Iterate through and write out all of the packets in the scans
  for (int i = 0; i < num_packets; ++i)
  {
    output_->sendPacket (scanMsg->packets[i]);
  }
}

} // namespace velodyne_driver
