// -*-c++-*--------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef BASALT_ROS__VIO_PUBLISHER_HPP_
#define BASALT_ROS__VIO_PUBLISHER_HPP_
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "basalt_ros/basalt_types.hpp"

namespace basalt_ros
{
class VIOPublisher
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit VIOPublisher(rclcpp::Node * node);

  void publish(const BasaltPoseVelBiasState::Ptr & data);
  // frame of the IMU (= basalt body frame), taken from the IMU messages
  void setChildFrameId(const std::string & f);

private:
  // fetch the static T_imu_base from TF, returns false if not available yet
  bool updateImuToBase();
  // compute the world alignment from the first pose, false if not ready yet
  bool updateStartPose(
    const Eigen::Vector3d & t_w_b, const Eigen::Quaterniond & q_w_b,
    int64_t t_ns);
  // ----- variables --
  rclcpp::Node * node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadCaster_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pub_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> pathPub_;
  std::array<double, 36> poseCov_;
  std::array<double, 36> twistCov_;
  nav_msgs::msg::Odometry msg_;
  std::vector<geometry_msgs::msg::PoseStamped> path_;  // full pose history
  bool extraTF_{false};
  bool publishTF_{false};
  Eigen::Vector3d T_extra_;     // extra translation:T_world_basaltworld
  Eigen::Quaterniond q_extra_;  // extra quaternion: q_world_basaltworld
  // optional output body frame (e.g. base_link). When set, odometry and TF
  // are published for this frame instead of the IMU frame.
  std::string baseFrame_;
  std::string imuFrame_;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  bool haveImuToBase_{false};
  Eigen::Vector3d t_i_b_{Eigen::Vector3d::Zero()};  // base origin in imu frame
  Eigen::Quaterniond q_i_b_{Eigen::Quaterniond::Identity()};
  // align_to_start: the first published pose defines the world frame
  // (origin and heading, gravity alignment kept), like the Isaac Sim ground
  // truth and cuVSLAM odometry that start at identity.
  bool alignToStart_{false};
  // align_frame (e.g. base_link_gt): the first pose is set to the TF pose
  // world_frame_id -> align_frame, so the odometry shares that reference
  std::string alignFrame_;
  bool haveStartPose_{false};
  Eigen::Vector3d t_start_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_start_{Eigen::Quaterniond::Identity()};
};
}  // namespace basalt_ros
#endif  // BASALT_ROS__VIO_PUBLISHER_HPP_
