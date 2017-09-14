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
  obst_sim_.reset (new ObstacleSimulator);

  // raw packet input topic
  input_ = node.subscribe<velodyne_msgs::VelodyneScan>("velodyne_packets", 10, &VelodyneSimulator::scanCallback, (VelodyneSimulator *) this);

  output_pub_ = node.advertise<velodyne_msgs::VelodyneScan>("velodyne_packets_simulated", 10);

  scan_counter_ = 0;
}


void VelodyneSimulator::updatePeople ()
{
  //ROS_ERROR_STREAM ("CALLBACK");
  obst_sim_->updatePeople();
  obst_sim_->updateBins();
}

void VelodyneSimulator::scanCallback(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
{
  // Get number of packets in scan
  int num_packets = scanMsg->packets.size();

  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  velodyne_msgs::VelodyneScanPtr scan(new velodyne_msgs::VelodyneScan);
  scan->packets.resize(num_packets);
  scan->header = scanMsg->header;

  if (scan_counter_ >= 10)
  {
    updatePeople ();
    scan_counter_ = 0;
  }
  //Iterate through and write out all of the packets in the scans
  for (int i = 0; i < num_packets; ++i)
  {
    //Do obstacle simulation
    velodyne_msgs::VelodynePacket::Ptr packet = obst_sim_->processPacket (scanMsg->packets[i]);

    //Send packet out as udp
    output_->sendPacket (*packet);
    
    //Also copy all packets for publisher
    scan->packets [i] = *packet;
  }

  output_pub_.publish(scan);
  scan_counter_++;
}

} // namespace velodyne_driver
