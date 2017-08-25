/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  Velodyne 3D LIDAR data input classes
 *
 *    These classes provide raw Velodyne LIDAR input packets from
 *    either a live socket interface or a previously-saved PCAP dump
 *    file.
 *
 *  Classes:
 *
 *     velodyne::Input -- base class for accessing the data
 *                      independently of its source
 *
 *     velodyne::InputSocket -- derived class reads live data from the
 *                      device via a UDP socket
 *
 *     velodyne::InputPCAP -- derived class provides a similar interface
 *                      from a PCAP dump file
 */

#ifndef __VELODYNE_OUTPUT_H
#define __VELODYNE_OUTPUT_H

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>

#include <ros/ros.h>
#include <velodyne_msgs/VelodynePacket.h>

namespace velodyne_driver
{
  static uint16_t DATA_PORT_NUMBER = 2368;     // default data port
  static uint16_t POSITION_PORT_NUMBER = 8308; // default position port

  /** @brief Velodyne output base class */
  class Output
  {
  public:
    Output(ros::NodeHandle private_nh, const std::string &ip_addr, uint16_t port);
    virtual ~Output() {}

    /** @brief Write one Velodyne packet.
     */
    virtual int sendPacket(const velodyne_msgs::VelodynePacket &pkt) = 0;

  protected:
    ros::NodeHandle private_nh_;
    uint16_t port_;
    std::string devip_str_;
  };

  /** @brief Live Velodyne input from socket. */
  class OutputSocket: public Output
  {
  public:
    OutputSocket(ros::NodeHandle private_nh,
                const std::string &ip_addr, uint16_t port = DATA_PORT_NUMBER);
    virtual ~OutputSocket();

    virtual int sendPacket(const velodyne_msgs::VelodynePacket &pkt);

  private:
    int sockfd_;
    in_addr devip_;
    sockaddr_in dest_address_;
    socklen_t dest_address_len_;
  };


} // velodyne_driver namespace

#endif // __VELODYNE_OUTPUT_H
