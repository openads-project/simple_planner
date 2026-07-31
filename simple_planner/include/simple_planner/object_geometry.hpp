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

/**
 * @brief Wraps an angle to the range [-pi, pi].
 *
 * @param[in] angle_rad Angle in radians.
 * @return Wrapped angle in radians.
 */
inline double wrap_angle_rad(double angle_rad) {
  double capped_angle_rad = angle_rad;
  while (capped_angle_rad > M_PI) capped_angle_rad -= 2 * M_PI;
  while (capped_angle_rad < -M_PI) capped_angle_rad += 2 * M_PI;
  return capped_angle_rad;
}

/**
 * @brief Rotates a 2D vector by the given yaw angle.
 *
 * @param[in] vec Vector to rotate.
 * @param[in] yaw Rotation angle in radians.
 * @return Rotated vector.
 */
inline Eigen::Vector2d rotate(const Eigen::Vector2d& vec, double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Eigen::Vector2d(c * vec.x() - s * vec.y(), s * vec.x() + c * vec.y());
}

/**
 * @brief Computes an object state's time relative to the planning stamp.
 *
 * @param[in] state Object state whose stamp should be used if available.
 * @param[in] fallback_header Header used when the state has no own stamp.
 * @param[in] planning_stamp Reference time of the planning cycle.
 * @return Relative time in seconds.
 */
inline double getStateRelativeTime(const perception_msgs::msg::ObjectState& state,
                                   const std_msgs::msg::Header& fallback_header,
                                   const rclcpp::Time& planning_stamp) {
  const bool has_state_stamp = state.header.stamp.sec != 0 || state.header.stamp.nanosec != 0;
  const auto& source_header = has_state_stamp ? state.header : fallback_header;
  return (rclcpp::Time(source_header.stamp) - planning_stamp).seconds();
}

/**
 * @brief Builds an oriented 2D bounding box from center pose and dimensions.
 *
 * @param[in] center Box center in the target frame.
 * @param[in] yaw Box heading in radians.
 * @param[in] length Box length.
 * @param[in] width Box width.
 * @return Oriented bounding box.
 */
inline OrientedBox2D buildOrientedBox(const Eigen::Vector2d& center, double yaw, double length, double width) {
  OrientedBox2D box;
  box.center = center;
  box.axis_x = rotate(Eigen::Vector2d::UnitX(), yaw);
  box.axis_y = rotate(Eigen::Vector2d::UnitY(), yaw);
  box.half_length = std::max(length, 0.0) / 2.0;
  box.half_width = std::max(width, 0.0) / 2.0;
  return box;
}

/**
 * @brief Expands a box by the given safety margins.
 *
 * @param[in] box Box to expand.
 * @param[in] longitudinal_safety_distance Longitudinal safety margin.
 * @param[in] lateral_safety_distance Lateral safety margin.
 * @return Expanded box.
 */
inline OrientedBox2D expandBoxWithSafetyMargins(const OrientedBox2D& box,
                                                double longitudinal_safety_distance,
                                                double lateral_safety_distance) {
  OrientedBox2D expanded_box = box;
  const double longitudinal_margin = std::max(longitudinal_safety_distance, -2.0 * box.half_length);
  const double lateral_margin = std::max(lateral_safety_distance, -box.half_width);
  expanded_box.center += 0.5 * longitudinal_margin * box.axis_x;
  expanded_box.half_length += 0.5 * longitudinal_margin;
  expanded_box.half_width += lateral_margin;
  return expanded_box;
}

/**
 * @brief Returns the corners of an oriented box.
 *
 * @param[in] box Box whose corners should be returned.
 * @return Box corners.
 */
inline std::array<Eigen::Vector2d, 4> getBoxCorners(const OrientedBox2D& box) {
  return {box.center + box.half_length * box.axis_x + box.half_width * box.axis_y,
          box.center + box.half_length * box.axis_x - box.half_width * box.axis_y,
          box.center - box.half_length * box.axis_x - box.half_width * box.axis_y,
          box.center - box.half_length * box.axis_x + box.half_width * box.axis_y};
}

/**
 * @brief Converts an oriented box center and heading to a ROS pose.
 *
 * @param[in] box Box to convert.
 * @return Pose at the box center with the box heading.
 */
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

/**
 * @brief Interpolates a timed box sample at the requested relative time.
 *
 * @param[in] lhs Earlier or first sample.
 * @param[in] rhs Later or second sample.
 * @param[in] t Relative time to sample.
 * @return Interpolated timed box.
 */
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

/**
 * @brief Checks whether two oriented boxes overlap.
 *
 * @param[in] first_box First bounding box.
 * @param[in] second_box Second bounding box.
 * @return true if the boxes overlap.
 * @return false if a separating axis exists.
 */
inline bool overlaps(const OrientedBox2D& first_box, const OrientedBox2D& second_box) {
  const Eigen::Vector2d center_delta = second_box.center - first_box.center;
  const std::array<Eigen::Vector2d, 4> axes = {first_box.axis_x, first_box.axis_y, second_box.axis_x, second_box.axis_y};
  for (const auto& axis : axes) {
    const double first_extent = first_box.half_length * std::abs(axis.dot(first_box.axis_x)) +
                                first_box.half_width * std::abs(axis.dot(first_box.axis_y));
    const double second_extent = second_box.half_length * std::abs(axis.dot(second_box.axis_x)) +
                                 second_box.half_width * std::abs(axis.dot(second_box.axis_y));
    if (std::abs(axis.dot(center_delta)) > first_extent + second_extent + 1e-6) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Builds a timed bounding-box sample for a single object state.
 *
 * @param[in] state Object state in the vehicle frame.
 * @param[in] fallback_header Header used if the state has no own stamp.
 * @param[in] stamp Planning stamp used as relative time reference.
 * @param[in] length Object length.
 * @param[in] width Object width.
 * @return Timed box sample for the object state.
 */
inline TimedBox2D buildObjectSample(const perception_msgs::msg::ObjectState& state,
                                    const std_msgs::msg::Header& fallback_header,
                                    const rclcpp::Time& stamp,
                                    double length,
                                    double width) {
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
