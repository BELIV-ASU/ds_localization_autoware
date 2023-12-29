#ifndef DS_LOCALIZATION_AUTOWARE_HPP__DS_LOCALIZATION_AUTOWARE_HPP_
#define DS_LOCALIZATION_AUTOWARE_HPP__DS_LOCALIZATION_AUTOWARE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <std_srvs/srv/set_bool.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <string>

namespace ds_localization_autoware
{
class DsLocalizationAutoware : public rclcpp::Node
{
public:
  explicit DsLocalizationAutoware(const rclcpp::NodeOptions & node_options);
  bool _initial_pose_estimated = false;

private:
  void callbackAutowareLocalization(geometry_msgs::msg::PoseWithCovarianceStamped oxts_pose);
  void callbackOrientation(geometry_msgs::msg::Quaternion msg);
  
  std::string _node_name = "ds_localization_autoware";
    
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr oxts_pose_autoware_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr orientation_sub_;
  
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _pub3;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr oxts_pose_autoware_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr _client_ekf_trigger;
  std::shared_ptr<tf2_ros::TransformBroadcaster> _br;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // Buffer that stores several seconds of transforms for easy lookup by the listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  

  geometry_msgs::msg::PoseWithCovarianceStamped autoware_pose;
  geometry_msgs::msg::Quaternion orientation;

};
}  // namespace ds_localization_autoware

#endif  // DS_LOCALIZATION_AUTOWARE_HPP__DS_LOCALIZATION_AUTOWARE_HPP_
