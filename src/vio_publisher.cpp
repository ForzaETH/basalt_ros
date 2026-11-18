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

#include <cmath>
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
  pathPub_ = node_->create_publisher<nav_msgs::msg::Path>("/basalt/path", 2);

  
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
  imuFrame_ = node_->declare_parameter("child_frame_id", "camera_imu");
  // If base_frame_id is set (e.g. "base_link"), the IMU pose estimated by
  // basalt is composed with the static base->imu transform from TF, and
  // odometry/TF are published as world_frame_id -> base_frame_id.
  baseFrame_ = node_->declare_parameter("base_frame_id", std::string(""));
  msg_.child_frame_id = baseFrame_.empty() ? imuFrame_ : baseFrame_;
  if (!baseFrame_.empty()) {
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    RCLCPP_INFO_STREAM(
      node_->get_logger(), "publishing odometry for base frame " << baseFrame_);
  }
  // Per REP-105 the odom->base_link transform belongs to the state
  // estimator (EKF), so broadcasting TF is off by default.
  publishTF_ = node_->declare_parameter("publish_tf", false);
  // basalt's world frame has its origin at the IMU and a heading set by the
  // gravity alignment at startup (arbitrary for an upside-down IMU). The
  // published pose can be re-anchored at the first pose:
  //  - align_frame (e.g. base_link_gt, the Isaac Sim ground truth): the first
  //    pose is set to the TF world_frame_id -> align_frame at that time, so
  //    basalt's odometry is expressed in the same reference (full 6 DoF).
  //  - align_to_start: position and yaw start at zero, roll and pitch stay
  //    gravity aligned. Ignored when align_frame is set.
  alignFrame_ = node_->declare_parameter("align_frame", std::string(""));
  alignToStart_ = node_->declare_parameter("align_to_start", false);
  if (!alignFrame_.empty()) {
    if (!tfBuffer_) {
      tfBuffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
      tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    }
    RCLCPP_INFO_STREAM(
      node_->get_logger(), "aligning the first pose to TF "
                             << msg_.header.frame_id << " -> " << alignFrame_);
  } else if (alignToStart_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "aligning the world frame to the first pose (zero position and yaw)");
  }
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

void VIOPublisher::setChildFrameId(const std::string & f)
{
  if (f != imuFrame_) {
    imuFrame_ = f;
    haveImuToBase_ = false;
  }
  if (baseFrame_.empty()) {
    msg_.child_frame_id = f;
  }
}

bool VIOPublisher::updateImuToBase()
{
  if (haveImuToBase_) {
    return (true);
  }
  try {
    // transform that maps points in base frame to imu frame: T_imu_base
    const auto tf =
      tfBuffer_->lookupTransform(imuFrame_, baseFrame_, tf2::TimePointZero);
    const auto & tr = tf.transform.translation;
    const auto & r = tf.transform.rotation;
    t_i_b_ = Eigen::Vector3d(tr.x, tr.y, tr.z);
    q_i_b_ = Eigen::Quaterniond(r.w, r.x, r.y, r.z).normalized();
    haveImuToBase_ = true;
    RCLCPP_INFO_STREAM(
      node_->get_logger(), "got " << imuFrame_ << " -> " << baseFrame_
                                  << ": t=" << t_i_b_.transpose()
                                  << " q=" << q_i_b_.coeffs().transpose());
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_STREAM_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "waiting for TF " << imuFrame_ << " -> " << baseFrame_ << ": "
                        << e.what());
  }
  return (haveImuToBase_);
}

bool VIOPublisher::updateStartPose(
  const Eigen::Vector3d & t_w_b, const Eigen::Quaterniond & q_w_b,
  int64_t t_ns)
{
  if (haveStartPose_) {
    return (true);
  }
  // reference pose of the body at the first pose: identity up to roll and
  // pitch (align_to_start), or the TF pose of align_frame
  Eigen::Vector3d t_ref = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_ref = Eigen::Quaterniond::Identity();
  if (!alignFrame_.empty()) {
    try {
      const auto tf = tfBuffer_->lookupTransform(
        msg_.header.frame_id, alignFrame_, rclcpp::Time(t_ns, RCL_ROS_TIME));
      const auto & tr = tf.transform.translation;
      const auto & r = tf.transform.rotation;
      t_ref = Eigen::Vector3d(tr.x, tr.y, tr.z);
      q_ref = Eigen::Quaterniond(r.w, r.x, r.y, r.z).normalized();
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_STREAM_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "waiting for TF " << msg_.header.frame_id << " -> " << alignFrame_
                          << " to align the odometry: " << e.what());
      return (false);
    }
    // T_ref_w = T_ref_b * T_w_b^-1
    q_start_ = (q_ref * q_w_b.inverse()).normalized();
  } else {
    // heading of the body x axis projected on the horizontal plane
    const Eigen::Matrix3d R = q_w_b.toRotationMatrix();
    const double yaw = std::atan2(R(1, 0), R(0, 0));
    q_start_ = Eigen::Quaterniond(
      Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()));
  }
  t_start_ = t_ref - q_start_ * t_w_b;
  haveStartPose_ = true;
  RCLCPP_INFO_STREAM(
    node_->get_logger(), "start pose: basalt t=" << t_w_b.transpose()
                           << " -> t=" << t_ref.transpose()
                           << " q(xyzw)=" << (q_start_ * q_w_b).coeffs().transpose());
  return (true);
}

void VIOPublisher::publish(const BasaltPoseVelBiasState::Ptr & data)
{
  if (!baseFrame_.empty() && !updateImuToBase()) {
    return;  // cannot express the pose in the base frame yet
  }
  // pose of the body (imu, or base if configured) in basalt's world frame
  Eigen::Vector3d T_orig = data->T_w_i.translation();
  Eigen::Quaterniond q_orig = data->T_w_i.unit_quaternion();
  Eigen::Vector3d vel_world = data->vel_w_i;  // linear vel, world frame
  if (!baseFrame_.empty()) {
    // T_w_b = T_w_i * T_i_b. The base velocity would also need the
    // omega x r lever-arm term, which is dropped since basalt does not
    // estimate omega (negligible for small IMU-to-base offsets).
    T_orig = T_orig + q_orig * t_i_b_;
    q_orig = q_orig * q_i_b_;
  }
  if (!alignFrame_.empty() || alignToStart_) {
    if (!updateStartPose(T_orig, q_orig, data->t_ns)) {
      return;  // reference pose not available yet
    }
    // rotate the world velocity too: the body-frame twist stays unchanged
    vel_world = q_start_ * vel_world;
    T_orig = q_start_ * T_orig + t_start_;
    q_orig = q_start_ * q_orig;
  }

  const Eigen::Vector3d T = extraTF_ ? (q_extra_ * T_orig + T_extra_) : T_orig;
  const Eigen::Quaterniond q = extraTF_ ? (q_extra_ * q_orig) : q_orig;

  // make odometry message
  msg_.header.stamp.sec = data->t_ns / 1000000000LL;
  msg_.header.stamp.nanosec = data->t_ns % 1000000000LL;

  msg_.pose.pose.position = to_ros_point(T);
  msg_.pose.pose.orientation = to_ros_quat(q);

  msg_.pose.covariance = poseCov_;

  // REP-103 / nav_msgs convention: twist is in the child (body) frame.
  // Rotate basalt's world-frame velocity into the body frame. This
  // is invariant under the extra world transform, so use the original q.
  const Eigen::Vector3d vel_body = q_orig.inverse() * vel_world;
  msg_.twist.twist.linear = to_ros_vec(vel_body);
  // basalt's state has no angular velocity; left at zero with large
  // covariance. Downstream consumers should take omega from the IMU.
  msg_.twist.covariance = twistCov_;

  pub_->publish(msg_);

  // Append to the pose history and publish the trajectory (as in MINS).

  // https://github.com/ros-visualization/rviz/issues/1107
  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg_.header;
  pose.pose = msg_.pose.pose;
  path_.push_back(pose);

  nav_msgs::msg::Path path;
  path.header.stamp = rclcpp::Clock().now();
  path.header.frame_id = msg_.header.frame_id;
  const size_t step =
    static_cast<size_t>(std::floor((double)path_.size() / 16384.0)) + 1;
  for (size_t i = 0; i < path_.size(); i += step) {
    path.poses.push_back(path_[i]);
  }
  pathPub_->publish(path);

  if (publishTF_) {
    // make transform message
    const rclcpp::Time t(msg_.header.stamp.sec, msg_.header.stamp.nanosec);
    const geometry_msgs::msg::TransformStamped tf =
      to_tf_msg(t, q, T, msg_.header.frame_id, msg_.child_frame_id);
    
    tfBroadCaster_->sendTransform(tf);
  }
}

}  // namespace basalt_ros
