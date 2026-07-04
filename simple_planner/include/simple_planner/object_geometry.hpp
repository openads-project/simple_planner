// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>

#include <perception_msgs/msg/object_state.hpp>
#include <perception_msgs_utils/object_access.hpp>

#include <rclcpp/time.hpp>

// Pure, node-independent geometry used by the object-handling logic (see object_handling.cpp).
// Everything here is free of ROS-node state and therefore unit-testable in isolation.

namespace simple_planner {

// 2D oriented bounding box used for object/ego conflict checks.
struct OrientedBox2D {
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  Eigen::Vector2d axis_x = Eigen::Vector2d::UnitX();
  Eigen::Vector2d axis_y = Eigen::Vector2d::UnitY();
  double half_length = 0.0;
  double half_width = 0.0;
};

// Oriented bounding box with the time (relative to the planning stamp) it refers to.
struct TimedBox2D {
  OrientedBox2D box;
  double t = 0.0;
  double yaw = 0.0;
  double length = 0.0;
  double width = 0.0;
};

// An object reduced to a sequence of timed boxes (single box if static).
struct ObjectTrajectory {
  uint64_t id = 0;
  bool is_static = false;
  std::vector<TimedBox2D> samples;
};

// A detected conflict between the ego trajectory and an object box.
struct ConflictSample {
  uint64_t object_id = 0;
  OrientedBox2D ego_box;
  OrientedBox2D object_box;
};

inline double wrap_angle_rad(double angle_rad) {
  double capped_angle_rad = angle_rad;
  while (capped_angle_rad > M_PI) capped_angle_rad -= 2 * M_PI;
  while (capped_angle_rad < -M_PI) capped_angle_rad += 2 * M_PI;
  return capped_angle_rad;
}

inline Eigen::Vector2d rotate(const Eigen::Vector2d& vec, double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Eigen::Vector2d(c * vec.x() - s * vec.y(), s * vec.x() + c * vec.y());
}

inline double getStateRelativeTime(const perception_msgs::msg::ObjectState& state, const std_msgs::msg::Header& fallback_header,
                                   const rclcpp::Time& planning_stamp) {
  const bool has_state_stamp = state.header.stamp.sec != 0 || state.header.stamp.nanosec != 0;
  const auto& source_header = has_state_stamp ? state.header : fallback_header;
  return (rclcpp::Time(source_header.stamp) - planning_stamp).seconds();
}

inline OrientedBox2D buildOrientedBox(const Eigen::Vector2d& center, double yaw, double length, double width) {
  OrientedBox2D box;
  box.center = center;
  box.axis_x = rotate(Eigen::Vector2d::UnitX(), yaw);
  box.axis_y = rotate(Eigen::Vector2d::UnitY(), yaw);
  box.half_length = std::max(length, 0.0) / 2.0;
  box.half_width = std::max(width, 0.0) / 2.0;
  return box;
}

inline geometry_msgs::msg::Pose toPose(const OrientedBox2D& box) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = box.center.x();
  pose.position.y = box.center.y();
  pose.position.z = 0.0;
  const double yaw = std::atan2(box.axis_x.y(), box.axis_x.x());
  pose.orientation.z = std::sin(yaw / 2.0);
  pose.orientation.w = std::cos(yaw / 2.0);
  return pose;
}

inline TimedBox2D interpolateTimedBox(const TimedBox2D& lhs, const TimedBox2D& rhs, double t) {
  const double duration = rhs.t - lhs.t;
  const double alpha = std::abs(duration) > 1e-6 ? std::clamp((t - lhs.t) / duration, 0.0, 1.0) : 0.0;
  TimedBox2D sample;
  sample.t = t;
  sample.yaw = wrap_angle_rad(lhs.yaw + alpha * wrap_angle_rad(rhs.yaw - lhs.yaw));
  sample.length = lhs.length + alpha * (rhs.length - lhs.length);
  sample.width = lhs.width + alpha * (rhs.width - lhs.width);
  const Eigen::Vector2d center = lhs.box.center + alpha * (rhs.box.center - lhs.box.center);
  sample.box = buildOrientedBox(center, sample.yaw, sample.length, sample.width);
  return sample;
}

// Separating-axis test between the (longitudinally/laterally inflated) ego box and an object box.
inline bool overlapsWithEgoSafety(const OrientedBox2D& ego_box, const OrientedBox2D& object_box,
                                  double longitudinal_safety_distance, double lateral_safety_distance) {
  const Eigen::Vector2d center_delta = object_box.center - ego_box.center;
  const std::array<Eigen::Vector2d, 4> axes = {ego_box.axis_x, ego_box.axis_y, object_box.axis_x, object_box.axis_y};
  for (const auto& axis : axes) {
    const double ego_extent = ego_box.half_length * std::abs(axis.dot(ego_box.axis_x)) +
                              ego_box.half_width * std::abs(axis.dot(ego_box.axis_y));
    const double object_extent = object_box.half_length * std::abs(axis.dot(object_box.axis_x)) +
                                 object_box.half_width * std::abs(axis.dot(object_box.axis_y));
    const double safety_extent = longitudinal_safety_distance * std::abs(axis.dot(ego_box.axis_x)) +
                                 lateral_safety_distance * std::abs(axis.dot(ego_box.axis_y));
    if (std::abs(axis.dot(center_delta)) > ego_extent + object_extent + safety_extent + 1e-6) {
      return false;
    }
  }
  return true;
}

// Builds a timed bounding box for a single object state in trajectory frame.
inline TimedBox2D buildObjectSample(const perception_msgs::msg::ObjectState& state, const std_msgs::msg::Header& fallback_header,
                                    const rclcpp::Time& stamp, double length, double width) {
  const auto center_msg = perception_msgs::object_access::getCenterPosition(state);
  TimedBox2D sample;
  sample.yaw = perception_msgs::object_access::getYaw(state);
  sample.length = length;
  sample.width = width;
  sample.box = buildOrientedBox(Eigen::Vector2d(center_msg.x, center_msg.y), sample.yaw, length, width);
  sample.t = getStateRelativeTime(state, fallback_header, stamp);
  return sample;
}

}  // namespace simple_planner
