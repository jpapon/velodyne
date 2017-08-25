/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  Input classes for the Velodyne HDL-64E 3D LIDAR:
 *
 *     Input -- base class used to access the data independently of
 *              its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 *
 *     InputPCAP -- derived class provides a similar interface from a
 *              PCAP dump
 */

#include <unistd.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <velodyne_driver/output.h>

namespace velodyne_driver
{

  ////////////////////////////////////////////////////////////////////////
  // Input base class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number.
   */
  Output::Output(ros::NodeHandle private_nh, const std::string &ip_addr, uint16_t port):
    private_nh_(private_nh),
    port_(port),
    devip_str_ (ip_addr)
  {

  }

  ////////////////////////////////////////////////////////////////////////
  // OutputSocket class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   */
  OutputSocket::OutputSocket(ros::NodeHandle private_nh, const std::string &ip_addr, uint16_t port):
    Output(private_nh, ip_addr, port)
  {
    sockfd_ = -1;
    
    if (!devip_str_.empty()) {
      inet_aton(devip_str_.c_str(),&devip_);
    }    

    // connect to Velodyne UDP port
    ROS_INFO_STREAM("Opening UDP socket: port " << port);
    sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
    if (sockfd_ == -1)
      {
        perror("socket");               // TODO: ROS_ERROR errno
        return;
      }
  
    memset(&dest_address_, 0, sizeof(dest_address_));    // initialize to zeros
    dest_address_.sin_family = AF_INET;            // host byte order
    dest_address_.sin_port = htons(port);          // port in network byte order
    dest_address_.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP
    dest_address_len_ = sizeof(dest_address_);

    if (bind(sockfd_, (sockaddr *)&dest_address_, sizeof(sockaddr)) == -1)
      {
        ROS_ERROR_STREAM("Failed to bind!");  
        return;
      }
  
    if (fcntl(sockfd_,F_SETFL, O_NONBLOCK|FASYNC) < 0)
      {
        ROS_ERROR_STREAM("Failed on non block!");  
        return;
      }

    dest_address_.sin_addr.s_addr = inet_addr(devip_str_.c_str());
    ROS_INFO_STREAM("Velodyne socket fd is "<< sockfd_);
  }

  /** @brief destructor */
  OutputSocket::~OutputSocket(void)
  {
    (void) close(sockfd_);
  }

  /** @brief Write one velodyne packet. */
  int OutputSocket::sendPacket(const velodyne_msgs::VelodynePacket &pkt)
  {
    const size_t packet_size = sizeof(velodyne_msgs::VelodynePacket().data);

    /* send a message to the server */
    if (sendto(sockfd_, &pkt.data[0], packet_size, 0, (sockaddr*) &dest_address_,  dest_address_len_) < 0) {
      ROS_ERROR_STREAM("sendto failed");
      return 1;
    }
    return 0;
  }



} // velodyne namespace
