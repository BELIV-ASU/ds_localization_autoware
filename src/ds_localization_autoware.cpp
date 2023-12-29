// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ds_localization_autoware/ds_localization_autoware.hpp"
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <cmath>



namespace ds_localization_autoware
{
DsLocalizationAutoware::DsLocalizationAutoware(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("ds_localization_autoware", node_options)

{
  oxts_pose_autoware_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("oxts_pose_ds",rclcpp::QoS{1},std::bind(&DsLocalizationAutoware::callbackAutowareLocalization, this, std::placeholders::_1));
  orientation_sub_ = create_subscription<geometry_msgs::msg::Quaternion>("oxts_orientation_autoware", rclcpp::QoS{1},std::bind(&DsLocalizationAutoware::callbackOrientation, this, std::placeholders::_1));
  
  oxts_pose_autoware_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("oxts_pose_autoware", rclcpp::QoS{1});
  _pub3 = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose3d", 1000);
  _client_ekf_trigger = create_client<std_srvs::srv::SetBool>("ekf_trigger_node");
  _br = std::make_shared<tf2_ros::TransformBroadcaster>(this,100);
}


void DsLocalizationAutoware::callbackOrientation(geometry_msgs::msg::Quaternion msg)
{
  orientation = msg;
}

void DsLocalizationAutoware::callbackAutowareLocalization(geometry_msgs::msg::PoseWithCovarianceStamped oxts_pose)
{

  autoware_pose.header = oxts_pose.header;
  autoware_pose.header.frame_id = "map";
  
  double utm_x = oxts_pose.pose.pose.position.x;
  double utm_y = oxts_pose.pose.pose.position.y;
  double utm_z = oxts_pose.pose.pose.position.z;
  
  // Convert the UTM coordinates to MGRS coordinates
  std::string utm_x_str = std::to_string(utm_x);
  std::string utm_y_str = std::to_string(utm_y);
  std::string utm_z_str = std::to_string(utm_z);
  
  double mgrs_x = std::stod(utm_x_str.substr(1));
  double mgrs_y = std::stod(utm_y_str.substr(2));
  double mgrs_z = std::stod(utm_z_str);
  
  autoware_pose.pose.pose.position.x = mgrs_x;
  autoware_pose.pose.pose.position.y = mgrs_y;
  autoware_pose.pose.pose.position.z = mgrs_z;
  
  // Keep UTM coordinates, 
  //autoware_pose.pose.pose.position.x = utm_x;
  //autoware_pose.pose.pose.position.y = utm_y;
  //autoware_pose.pose.pose.position.z = utm_z;
  
  // Keep UTM orientation
  autoware_pose.pose.pose.orientation = oxts_pose.pose.pose.orientation;
  // Renormarlize the quaternion

  autoware_pose.pose.covariance = oxts_pose.pose.covariance;
  //autoware_pose.pose.covariance = {0.0225,0,0,0,0,0,0,0.0225,0,0,0,0,0,0,0.0225,0,0,0,0,0,0,0.000625,0,0,0,0,0,0,0.000625,0,0,0,0,0,0,0.000625}; // ndt covariance settings
  //autoware_pose.pose.covariance = {10000,0,0,0,0,0,0,2.25,0,0,0,0,0,0,2.25,0,0,0,0,0,0,7.615435494667714e-05,0,0,0,0,0,0,10000.0,0,0,0,0,0,0,0.001}; //eagleye covariance setting
  autoware_pose.pose.covariance[21] = 0.000625;
  autoware_pose.pose.covariance[28] = 0.000625;
  autoware_pose.pose.covariance[35] = 0.000625;
  
  geometry_msgs::msg::PoseStamped target_pose_in_mgrs_frame;
  target_pose_in_mgrs_frame.header = autoware_pose.header;
  target_pose_in_mgrs_frame.pose = autoware_pose.pose.pose;

  geometry_msgs::msg::PoseStamped target_pose_in_local_frame;
  tf_buffer_->transform<geometry_msgs::msg::PoseStamped>(target_pose_in_mgrs_frame, target_pose_in_local_frame, "new_map",
                tf2::Duration(std::chrono::seconds(1)));
 
  autoware_pose.pose.pose = target_pose_in_local_frame.pose;
  
  oxts_pose_autoware_pub_->publish(autoware_pose);
  
  // publish tf 
  tf2::Transform transform;
  tf2::Quaternion q;
  transform.setOrigin(tf2::Vector3(autoware_pose.pose.pose.position.x, autoware_pose.pose.pose.position.y, autoware_pose.pose.pose.position.z));
  q.setX(autoware_pose.pose.pose.orientation.x);
  q.setY(autoware_pose.pose.pose.orientation.y);
  q.setZ(autoware_pose.pose.pose.orientation.z);
  q.setW(autoware_pose.pose.pose.orientation.w);
  transform.setRotation(q);

  geometry_msgs::msg::TransformStamped trans_msg;
  trans_msg.header.stamp = autoware_pose.header.stamp;
  trans_msg.header.frame_id = "map";
  trans_msg.child_frame_id = "ds_base_link";
  trans_msg.transform = tf2::toMsg(transform);
  _br->sendTransform(trans_msg);
}



  
} 

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ds_localization_autoware::DsLocalizationAutoware)
