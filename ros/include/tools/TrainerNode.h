/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBSLAM2_ROS_TRAINER_NODE_H_
#define ORBSLAM2_ROS_TRAINER_NODE_H_

#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>
#include <sensor_msgs/Image.h>
#include "orb_slam2_ros/SaveMap.h"


class TrainerNode
{
  public:
    TrainerNode ();
    ~TrainerNode ();
    void isGBARunningCallback(const std_msgs::Bool::ConstPtr& isGBARunningMsg);
    void update(void);

  private:
    ros::Subscriber status_GBA_sub;
    ros::Publisher color_image_pub;
    ros::Publisher depth_image_pub;
    ros::ServiceClient orb2_save_map_service;

    rosbag::Bag input_bag;
    std::string video_rosbag_name;
    std::string color_image_topic_name;
    std::string depth_image_topic_name;
    std::string map_file_name;
    int training_loops = 1;
    bool do_reverse_video = false;

    std::vector<rosbag::View::iterator> color_image_iters;
    std::vector<rosbag::View::iterator>::iterator color_image_fwd_iterator;
    std::vector<rosbag::View::iterator>::reverse_iterator color_image_bwd_iterator;
    std::vector<rosbag::View::iterator> depth_image_iters;
    std::vector<rosbag::View::iterator>::iterator depth_image_fwd_iterator;
    std::vector<rosbag::View::iterator>::reverse_iterator depth_image_bwd_iterator;

    sensor_msgs::Image::Ptr color_image_msg;
    sensor_msgs::Image::Ptr depth_image_msg;

    bool training_active =  true;
    bool is_GBA_running = false;
    bool is_GBA_running_previous = false;
    bool playing_reverse_video = false;
    int loop_number = 1;
};

#endif //ORBSLAM2_ROS_TRAINER_NODE_H_
