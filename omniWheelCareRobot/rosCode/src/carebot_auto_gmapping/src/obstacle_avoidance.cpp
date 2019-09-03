/******************************************************************************
   STDR Simulator - Simple Two DImensional Robot Simulator
   Copyright (C) 2013 STDR Simulator
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
   
   Authors : 
   * Manos Tsardoulias, etsardou@gmail.com
   * Aris Thallas, aris.thallas@gmail.com
   * Chris Zalidis, zalidis@gmail.com 
******************************************************************************/
# include "obstacle_avoidance.h"

/**
@namespace stdr_samples
@brief The main namespace for STDR Samples
**/ 
namespace stdr_samples
{
  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  ObstacleAvoidance::ObstacleAvoidance(std::string cmd_topic, std::string laser_topic)
  {
    speeds_topic_ = cmd_topic;
    laser_topic_  = laser_topic;
      
    subscriber_ = n_.subscribe(
      laser_topic_.c_str(), 
      1, 
      &ObstacleAvoidance::callback,
      this);
      
    cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>(speeds_topic_.c_str(), 1);
  }
  
  /**
  @brief Default destructor
  @return void
  **/
  ObstacleAvoidance::~ObstacleAvoidance(void)
  {
    
  }
  
  /**
  @brief Callback for the ros laser message
  @param msg [const sensor_msgs::LaserScan&] The new laser scan message
  @return void
  **/
  void ObstacleAvoidance::callback(const sensor_msgs::LaserScan& msg)
  {
    scan_ = msg;
    float linear = 0, rotational = 0;

    for(unsigned int i = 0 ; i < scan_.ranges.size() ; i++)
    {
      float real_dist = scan_.ranges[i];
      linear -= cos(scan_.angle_min + i * scan_.angle_increment) 
                   / (1.0 + real_dist * real_dist);
      rotational -= sin(scan_.angle_min + i * scan_.angle_increment) 
                    / (1.0 + real_dist * real_dist);
    }
    
    linear /= scan_.ranges.size();
    rotational /= scan_.ranges.size();
    ROS_WARN("%f %f",linear,rotational);
    
    if(linear > 0.1)
    {
      linear = 0.1;
    }
    else if(linear < -0.1)
    {
      linear = -0.1;
    }

    geometry_msgs::Twist cmd;
    cmd.linear.x  = 0.1 + linear;
    cmd.angular.z = rotational;
    cmd_vel_pub_.publish(cmd);
  }

}


