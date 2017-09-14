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

#ifndef _OBSTACLE_SIMULATOR_H_
#define _OBSTACLE_SIMULATOR_H_

#include <random>
#include <string>
#include <ros/ros.h>

#include <velodyne_driver/output.h>
#include <velodyne_msgs/VelodyneScan.h>

namespace velodyne_driver
{

/**
   * Raw Velodyne packet constants and structures.
   */
  static const int SIZE_BLOCK = 100;
  static const int RAW_SCAN_SIZE = 3;
  static const int SCANS_PER_BLOCK = 32;
  static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

  static const float ROTATION_RESOLUTION      =     0.01f;  // [deg]
  static const uint16_t ROTATION_MAX_UNITS    = 36000u;     // [deg/100]
  static const float DISTANCE_RESOLUTION      =     0.002f; // [m]

  /** @todo make this work for both big and little-endian machines */
  static const uint16_t UPPER_BANK = 0xeeff;
  static const uint16_t LOWER_BANK = 0xddff;
  
  
  /** Special Defines for VLP16 support **/
  static const int    VLP16_FIRINGS_PER_BLOCK =   2;
  static const int    VLP16_SCANS_PER_FIRING  =  16;
  static const float  VLP16_BLOCK_TDURATION   = 110.592f;   // [µs]
  static const float  VLP16_DSR_TOFFSET       =   2.304f;   // [µs]
  static const float  VLP16_FIRING_TOFFSET    =  55.296f;   // [µs]
  
  static const float MAX_DIST = 200.0f;

  /** \brief Raw Velodyne data block.
   *
   *  Each block contains data from either the upper or lower laser
   *  bank.  The device returns three times as many upper bank blocks.
   *
   *  use stdint.h types, so things work with both 64 and 32-bit machines
   */
  typedef struct raw_block
  {
    uint16_t header;        ///< UPPER_BANK or LOWER_BANK
    uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees
    uint8_t  data[BLOCK_DATA_SIZE];
  } raw_block_t;

  /** used for unpacking the first two data bytes in a block
   *
   *  They are packed into the actual data stream misaligned.  I doubt
   *  this works on big endian machines.
   */
  union two_bytes
  {
    uint16_t uint;
    uint8_t  bytes[2];
  };

  static const int PACKET_SIZE = 1206;
  static const int BLOCKS_PER_PACKET = 12;
  static const int PACKET_STATUS_SIZE = 4;
  static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

  /** \brief Raw Velodyne packet.
   *
   *  revolution is described in the device manual as incrementing
   *    (mod 65536) for each physical turn of the device.  Our device
   *    seems to alternate between two different values every third
   *    packet.  One value increases, the other decreases.
   */
  typedef struct raw_packet
  {
    raw_block_t blocks[BLOCKS_PER_PACKET];
    uint16_t revolution;
    uint8_t status[PACKET_STATUS_SIZE]; 
  } raw_packet_t;

struct Person{
  float width;
  float range;
  int direction;
  float leg_slope;
  int leg_slope_direction;
  int azimuth;
  int az_bins;

};


class ObstacleSimulator
{
public:
  typedef boost::shared_ptr<ObstacleSimulator> Ptr;
  ObstacleSimulator(int num_people);
  ~ObstacleSimulator() {}

  velodyne_msgs::VelodynePacket::Ptr
  processPacket(const velodyne_msgs::VelodynePacket &msg);

  void
  updatePeople ();

  void 
  updateBins ();

  void 
  spawnPeople (int num_people);
protected:

  Person
  spawnPerson ();

  void
  modifyPacket(const velodyne_msgs::VelodynePacket &pkt, velodyne_msgs::VelodynePacket &pkt_out);




private:
  std::vector<float> azimuth_bins_;

  std::vector<Person> people_;

  std::mt19937 gen_;
  std::uniform_int_distribution<> az_dis_;
  std::uniform_int_distribution<> direction_dis_;

  std::uniform_real_distribution<> width_dis_;
  std::uniform_real_distribution<> range_dis_;

  std::uniform_int_distribution<> az_delta_dis_;
  std::uniform_real_distribution<> range_delta_dis_;

};

} // namespace velodyne_driver
#endif // _OBSTACLE_SIMULATOR_H_
