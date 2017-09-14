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

ObstacleSimulator::ObstacleSimulator(int num_people)
{
  azimuth_bins_.resize(36000, MAX_DIST);
  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  gen_= std::mt19937(rd()); //Standard mersenne_twister_engine seeded with rd()
  az_dis_ = std::uniform_int_distribution<>(0, 35999);
  width_dis_= std::uniform_real_distribution<>(0.5, 0.65); //in meters- width of person
  range_dis_ = std::uniform_real_distribution<>(3.0, 15.0);
  direction_dis_= std::uniform_int_distribution<>(1, 100);
  az_delta_dis_ = std::uniform_int_distribution<>(100, 300);
  range_delta_dis_ = std::uniform_real_distribution<>(0.1, 0.3);

  spawnPeople(num_people);
  updatePeople ();
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

  //DEBUGGING
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
    int az_bins = people_[idx].az_bins;
    float range = people_[idx].range;
    float leg_slope = people_[idx].leg_slope;

    int gap_start = az + 2*az_bins/5;
    int gap_end = az + 3*az_bins/5;

    float slope_delta_per_bin = leg_slope * 0.5/az_bins;
    float slope_delta = 0;
    for (int k = az; k < az + az_bins; ++k)
    {
      if (k < gap_start or k >= gap_end)
      {
        int az_bin = k % 36000;
        azimuth_bins_[az_bin] = range + slope_delta;
      }
      slope_delta += slope_delta_per_bin;
    }

    /*
    //Check if we need to wrap.
    int num_wrapped = (az + az_bins) - azimuth_bins_.size();
    //If num wrapped is positive, we need to fill that many bins starting from beginning
    if (num_wrapped > 0)
    {
      std::fill_n (azimuth_bins_.begin(), num_wrapped, range);
      az_bins -= num_wrapped;
    }

    //Fill the rest starting from az - split in galf leaving a gap of 1/5th in middle
    std::fill_n (azimuth_bins_.begin() + az, 2*az_bins/5, range);
    std::fill_n (azimuth_bins_.begin() + az+3*az_bins/5, 2*az_bins/5, range);

    */
  }
}


void
ObstacleSimulator::updatePeople ()
{
  for (int i = 0; i < people_.size(); ++i)
  {
    int az = people_[i].azimuth;
    float range = people_[i].range;
    int direction = people_[i].direction;
    float leg_slope = people_[i].leg_slope;
    int leg_slope_direction = people_[i].leg_slope_direction;

    az += az_delta_dis_ (gen_) * direction;
    if (az < 0)
      az += 36000;
    az = az % 36000;
    
    //small random chance to switch direction direction_dis_(gen_) is 1-100
    if (direction_dis_(gen_) > 96)
      direction *= -1;
    //If at limits, switch direction
    if (range <= 1.5)
      direction = 1;
    if (range >= 20.0)
      direction = -1;
  
    if (leg_slope >= 1.0)
      leg_slope_direction = -1;
    else if (leg_slope <= -1.0)
      leg_slope_direction = 1;

    leg_slope += leg_slope_direction * 0.5;

    range += range_delta_dis_(gen_) * direction;

    people_[i].azimuth = az;
    people_[i].range = range;
    people_[i].direction = direction;
    people_[i].leg_slope_direction = leg_slope_direction;
    people_[i].leg_slope = leg_slope;

    //Az bins is determined by range and width
    float width = people_[i].width;
    float angle_rads = 2*std::asin(width/(2*range));
    people_[i].az_bins = (180.0/M_PI) * angle_rads * 100;
    //ROS_ERROR_STREAM ("w="<<width<<"  r="<<range<<"  angle_deg ="<< (180.0/M_PI) * angle_rads<<"  bins= "<<people_[i].az_bins);
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
  new_person.width = width_dis_(gen_);
  new_person.range = range_dis_(gen_);
  new_person.direction = 2*(direction_dis_(gen_) % 2) - 1; //-1 or 1
  new_person.leg_slope = (direction_dis_(gen_) - 50) / 50.0f;  //-1 to 1
  new_person.leg_slope_direction = 2*(direction_dis_(gen_) % 2) - 1; //-1 or 1

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

  pkt_out = pkt;
  raw_packet_t *raw_out = (raw_packet_t *) &pkt_out.data[0];
  for (int block = 0; block < BLOCKS_PER_PACKET; block++)
  {
    // Calculate difference between current and next block's azimuth angle.
    azimuth = (float)(raw_out->blocks[block].rotation);
    if (block < (BLOCKS_PER_PACKET-1))
    {
      azimuth_diff = (float)((36000 + raw_out->blocks[block+1].rotation - raw_out->blocks[block].rotation)%36000);
      last_azimuth_diff = azimuth_diff;
    }else
    {
      azimuth_diff = last_azimuth_diff;
    }

    for (int firing=0, k=0; firing < VLP16_FIRINGS_PER_BLOCK; firing++)
    {
      for (int dsr=0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k+=RAW_SCAN_SIZE)
      {

        // Position Calculation
        union two_bytes tmp;
        tmp.bytes[0] = raw_out->blocks[block].data[k];
        tmp.bytes[1] = raw_out->blocks[block].data[k+1];
        
        // correct for the laser rotation as a function of timing during the firings
        azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*VLP16_DSR_TOFFSET) + (firing*VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
        azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;

        float distance = tmp.uint * DISTANCE_RESOLUTION;

        if (distance > azimuth_bins_[azimuth_corrected])
        {
          float out_dist = azimuth_bins_[azimuth_corrected];
          tmp.uint = out_dist / DISTANCE_RESOLUTION;
          raw_out->blocks[block].data[k] = tmp.bytes[0];
          raw_out->blocks[block].data[k+1] = tmp.bytes[1];
        }

      }
    }
    

  }

}  


} // namespace velodyne_driver
