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

#include "basalt_ros/vio_publisher.hpp"

#include <fstream>
#include <ios>
#include <stdexcept>

static geometry_msgs::msg::Point to_ros_point(const Eigen::Vector3d & v)
{
  geometry_msgs::msg::Point vv;
  vv.x = v[0];
  vv.y = v[1];
  vv.z = v[2];
  return (vv);
}

static geometry_msgs::msg::Vector3 to_ros_vec(const Eigen::Vector3d & v)
{
  geometry_msgs::msg::Vector3 vv;
  vv.x = v[0];
  vv.y = v[1];
  vv.z = v[2];
  return (vv);
}

static geometry_msgs::msg::Quaternion to_ros_quat(const Eigen::Quaterniond & q)
{
  geometry_msgs::msg::Quaternion qq;
  qq.x = q.x();
  qq.y = q.y();
  qq.z = q.z();
  qq.w = q.w();
  return (qq);
}

static geometry_msgs::msg::TransformStamped to_tf_msg(
  const rclcpp::Time & t, const Eigen::Quaterniond & q,
  const Eigen::Vector3d & trans, const std::string & parent,
  const std::string & child)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = t;
  tf.header.frame_id = parent;
  tf.child_frame_id = child;

  tf.transform.translation = to_ros_vec(trans);
  tf.transform.rotation = to_ros_quat(q);
  return (tf);
}

static std::array<double, 36> diag_cov(
  rclcpp::Node * node, const std::string & key,
  const std::vector<double> & def)
{
  const std::vector<double> d = node->declare_parameter(key, def);
  if (d.size() != 6) {
    RCLCPP_ERROR_STREAM(node->get_logger(), key << " must have 6 elements!");
    throw std::invalid_argument(key + " must have 6 elements");
  }
  std::array<double, 36> cov{};
  for (int i = 0; i < 6; i++) {
    cov[i * 7] = d[i];
  }
  return (cov);
}

namespace basalt_ros
{
VIOPublisher::VIOPublisher(rclcpp::Node * node) : node_(node)
{
  RCLCPP_INFO(node_->get_logger(), "starting publisher");

  pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  // we don't do covariance quite yet....
  // Diagonal covariances, overridable via parameters. Defaults are
  // deliberately conservative so a downstream EKF does not trust the
  // VIO more than its own motion model.
  poseCov_ = diag_cov(node_, "pose_covariance_diagonal",
                      {0.05, 0.05, 0.10, 0.05, 0.05, 0.05});
  // angular twist is NOT estimated by basalt (left at zero), so its
  // covariance is set very large by default
  twistCov_ = diag_cov(node_, "twist_covariance_diagonal",
                       {0.02, 0.02, 0.04, 1e3, 1e3, 1e3});
  msg_.header.frame_id = node_->declare_parameter("world_frame_id", "odom");
  msg_.child_frame_id =
    node_->declare_parameter("child_frame_id", "camera_imu");
  // Per REP-105 the odom->base_link transform belongs to the state
  // estimator (EKF), so broadcasting TF is off by default.
  publishTF_ = node_->declare_parameter("publish_tf", false);
  if (publishTF_) {
    tfBroadCaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  }
  std::vector<double> ext_trans = {0, 0, 0};
  std::vector<double> ext_q = {1.0, 0, 0, 0};

  if (!node_->has_parameter("extra_translation")) {
    ext_trans = node_->declare_parameter("extra_translation", ext_trans);
  }
  if (node_->get_parameter<std::vector<double>>(
        "extra_translation", ext_trans)) {
    if (ext_trans.size() != 3) {
      RCLCPP_ERROR(
        node->get_logger(), "extra_translation must have 3 elements!");
      throw std::invalid_argument("extra translation must have 3 elements");
    }
  }
  // rotation is given in format (w, x, y, z)
  if (!node_->has_parameter("extra_rotation")) {
    ext_q = node_->declare_parameter("extra_rotation", ext_q);
  }
  if (node_->get_parameter<std::vector<double>>("extra_rotation", ext_q)) {
    if (ext_q.size() != 4) {
      RCLCPP_ERROR(node->get_logger(), "extra_rotation must have 4 elements!");
      throw std::invalid_argument("extra rotation must have 4 elements");
    }
  }

  T_extra_ = Eigen::Vector3d(ext_trans[0], ext_trans[1], ext_trans[2]);
  q_extra_ = Eigen::Quaterniond(ext_q[0], ext_q[1], ext_q[2], ext_q[3]);
  extraTF_ = T_extra_.norm() > 0.001 || q_extra_.w() < 0.99999;
  if (extraTF_) {
    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "extra transform: q=" << q_extra_.coeffs().transpose()
                            << ", T=" << T_extra_.transpose());
  } else {
    RCLCPP_INFO(node->get_logger(), "no extra transform specified");
  }
}

void VIOPublisher::publish(const BasaltPoseVelBiasState::Ptr & data)
{
  const Eigen::Vector3d T_orig = data->T_w_i.translation();
  const Eigen::Quaterniond q_orig = data->T_w_i.unit_quaternion();
  const Eigen::Vector3d vel_world = data->vel_w_i;  // linear vel, world frame

  const Eigen::Vector3d T = extraTF_ ? (q_extra_ * T_orig + T_extra_) : T_orig;
  const Eigen::Quaterniond q = extraTF_ ? (q_extra_ * q_orig) : q_orig;

  // make odometry message
  msg_.header.stamp.sec = data->t_ns / 1000000000LL;
  msg_.header.stamp.nanosec = data->t_ns % 1000000000LL;

  msg_.pose.pose.position = to_ros_point(T);
  msg_.pose.pose.orientation = to_ros_quat(q);

  msg_.pose.covariance = poseCov_;

  // REP-103 / nav_msgs convention: twist is in the child (body) frame.
  // Rotate basalt's world-frame velocity into the IMU body frame. This
  // is invariant under the extra world transform, so use the original q.
  const Eigen::Vector3d vel_body = q_orig.inverse() * vel_world;
  msg_.twist.twist.linear = to_ros_vec(vel_body);
  // basalt's state has no angular velocity; left at zero with large
  // covariance. Downstream consumers should take omega from the IMU.
  msg_.twist.covariance = twistCov_;

  pub_->publish(msg_);

  if (publishTF_) {
    // make transform message
    const rclcpp::Time t(msg_.header.stamp.sec, msg_.header.stamp.nanosec);
    const geometry_msgs::msg::TransformStamped tf =
      to_tf_msg(t, q, T, msg_.header.frame_id, msg_.child_frame_id);
    
    tfBroadCaster_->sendTransform(tf);
  }
}

}  // namespace basalt_ros
