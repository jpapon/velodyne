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

#include "obstacle_simulator.h"

namespace velodyne_driver
{

ObstacleSimulator::ObstacleSimulator()
{
  azimuth_bins_.resize(36000, MAX_DIST);
  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  gen_= std::mt19937(rd()); //Standard mersenne_twister_engine seeded with rd()
  az_dis_ = std::uniform_int_distribution<>(0, 35999);
  size_dis_= std::uniform_int_distribution<>(500, 2000); //in degrees / 1000
  range_dis_ = std::uniform_real_distribution<>(1.0, 5.0);
  
  az_delta_dis_ = std::uniform_int_distribution<>(-1000, 1000);
  range_delta_dis_ = std::uniform_real_distribution<>(-0.2, 0.2);

  spawnPeople(25);

  //Update the azimuth bins based off of current people
  updateBins();
}


velodyne_msgs::VelodynePacket::Ptr
ObstacleSimulator::processPacket(const velodyne_msgs::VelodynePacket &msg)
{
  //Initialize returned packet.
  velodyne_msgs::VelodynePacket::Ptr output_packet (new velodyne_msgs::VelodynePacket);


  //Go through each beam and check for collisions
  modifyPacket (msg, *output_packet);
  //Set to collision distance if occurred, otherwise set to original range.



  //DEBUGGING
  //output_packet->data = msg.data;
  output_packet->stamp = msg.stamp;


  return output_packet;
}

void
ObstacleSimulator::updateBins ()
{
  //Set azimuth bins to max
  std::fill(azimuth_bins_.begin(),azimuth_bins_.end(), MAX_DIST);

  //Sort people based off of their range
  std::vector <std::pair<float,int>> people_range_indices;
  for (int i = 0; i < people_.size(); ++i)
    people_range_indices.push_back(std::make_pair(people_[i].range, i));

  struct {
      bool operator()(std::pair<float,int> a, std::pair<float,int> b) const
      {   
          return a.first > b.first;
      }   
  } pairGreater;

  std::sort (people_range_indices.begin(), people_range_indices.end(),  pairGreater );

  //Go through and update bins for each person - this is sorted by range, furthest is first so it will get overwritten by closer.
  for (int i = 0; i < people_range_indices.size(); ++i)
  {
    int idx = people_range_indices[i].second;
    int az = people_[idx].azimuth;
    int size = people_[idx].size;
    float range = people_[idx].range;

    //Check if we need to wrap.
    int num_wrapped = (az + size) - azimuth_bins_.size();
    //If num wrapped is positive, we need to fill that many bins starting from beginning
    if (num_wrapped > 0)
    {
      std::fill_n (azimuth_bins_.begin(), num_wrapped, range);
      size -= num_wrapped;
    }

    //Fill the rest starting from az
    std::fill_n (azimuth_bins_.begin() + az, size, range);
  }
}


void
ObstacleSimulator::updatePeople ()
{
  for (int i = 0; i < people_.size(); ++i)
  {
    int az = people_[i].azimuth;
    int size = people_[i].size;
    float range = people_[i].range;

    az += az_delta_dis_ (gen_);
    if (az < 0)
      az += 36000;
    az = az % 36000;
    
    if (range < 1.5)
      range += std::abs(range_delta_dis_(gen_));
    else
      range += range_delta_dis_(gen_);

    people_[i].azimuth = az;
    people_[i].range = range;

  }
}


void
ObstacleSimulator::spawnPeople (int num_people)
{
  people_.clear();
  //Set up some random people
  for (int i = 0; i < num_people; ++i)
  {
    people_.push_back (spawnPerson ());
  }
}

Person
ObstacleSimulator::spawnPerson ()
{
  Person new_person;
  new_person.azimuth = az_dis_(gen_);
  new_person.size = size_dis_(gen_);
  new_person.range = range_dis_(gen_);
  return new_person;
}


void
ObstacleSimulator::modifyPacket(const velodyne_msgs::VelodynePacket &pkt, velodyne_msgs::VelodynePacket &pkt_out)
{
  float azimuth;
  float azimuth_diff;
  float last_azimuth_diff=0;
  float azimuth_corrected_f;
  int azimuth_corrected;
  float x, y, z;
  float intensity;

  const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];
  raw_packet_t *raw_out = (raw_packet_t *) &pkt_out.data[0];
  for (int block = 0; block < BLOCKS_PER_PACKET; block++)
  {
    // Calculate difference between current and next block's azimuth angle.
    azimuth = (float)(raw->blocks[block].rotation);
    if (block < (BLOCKS_PER_PACKET-1))
    {
      azimuth_diff = (float)((36000 + raw->blocks[block+1].rotation - raw->blocks[block].rotation)%36000);
      last_azimuth_diff = azimuth_diff;
    }else
    {
      azimuth_diff = last_azimuth_diff;
    }


    //Set output to current.
    raw_out->blocks[block] = raw->blocks[block];
    for (int firing=0, k=0; firing < VLP16_FIRINGS_PER_BLOCK; firing++)
    {
      for (int dsr=0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k+=RAW_SCAN_SIZE)
      {

        /** Position Calculation */
        union two_bytes tmp;
        tmp.bytes[0] = raw->blocks[block].data[k];
        tmp.bytes[1] = raw->blocks[block].data[k+1];
        
        /** correct for the laser rotation as a function of timing during the firings **/
        azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*VLP16_DSR_TOFFSET) + (firing*VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
        azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;

        float distance = tmp.uint * DISTANCE_RESOLUTION;

        if (distance > azimuth_bins_[azimuth_corrected])
        {
          float out_dist = azimuth_bins_[azimuth_corrected];
          union two_bytes out_bytes;
          out_bytes.uint = out_dist / DISTANCE_RESOLUTION;
          raw_out->blocks[block].data[k] = out_bytes.bytes[0];
          raw_out->blocks[block].data[k+1] = out_bytes.bytes[1];
        }

      }
    }

  }

}  


} // namespace velodyne_driver
