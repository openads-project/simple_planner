// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2/utils.h>

#include <simple_planner/simple_planner.hpp>

namespace simple_planner {

Eigen::Vector2d SimplePlannerNode::transformToGridLocal(const Eigen::Vector2d& point_in_grid_frame,
                                                        const Eigen::Vector2d& grid_origin,
                                                        double grid_origin_yaw) {
  return rotate(point_in_grid_frame - grid_origin, -grid_origin_yaw);
}

Eigen::Vector2d SimplePlannerNode::transformToGridFrame(const Eigen::Vector2d& point_in_grid_local,
                                                        const Eigen::Vector2d& grid_origin,
                                                        double grid_origin_yaw) {
  return grid_origin + rotate(point_in_grid_local, grid_origin_yaw);
}

bool SimplePlannerNode::hasValidGridMap(const rclcpp::Time& stamp) const {
  if (!grid_map_init_) {
    return false;
  }

  const bool has_valid_geometry = std::isfinite(grid_map_.info.resolution) && grid_map_.info.resolution > 0.0 &&
                                  grid_map_.info.width > 0 && grid_map_.info.height > 0;
  const size_t expected_cell_count = static_cast<size_t>(grid_map_.info.width) * static_cast<size_t>(grid_map_.info.height);
  const bool has_complete_data = grid_map_.data.size() == expected_cell_count;

  return has_valid_geometry && has_complete_data && !isMessageOutdated(grid_map_.header, grid_map_timeout_, stamp);
}

void SimplePlannerNode::applyGridMapConstraints(const std_msgs::msg::Header& target_header,
                                                std::vector<SimplePathPoint>& base_path_points,
                                                FollowRoutePlan& route_plan) {
  if (!consider_grid_map_ || base_path_points.empty()) {
    return;
  }

  const auto obstacle_s = findFirstGridMapStopS(target_header, base_path_points);
  if (!obstacle_s.has_value()) {
    return;
  }

  const double stop_s = std::max(*obstacle_s, 0.0);
  base_path_points = truncatePathAtS(base_path_points, stop_s);
  route_plan.stop_at_end = true;
  route_plan.offset_to_stop_line = 0.0;
  route_plan.reason_to_stop = "Grid map obstacle";

  RCLCPP_INFO(this->get_logger(), "Applying stop for grid-map obstacle at first unsafe s=%f m", stop_s);
}

std::optional<double> SimplePlannerNode::findFirstGridMapStopS(const std_msgs::msg::Header& target_header,
                                                               const std::vector<SimplePathPoint>& base_path_points) {
  if (base_path_points.size() < 2) {
    return std::nullopt;
  }

  const geometry_msgs::msg::TransformStamped tf =
      tf2_buffer_->lookupTransform(grid_map_.header.frame_id, grid_map_.header.stamp, target_header.frame_id, target_header.stamp,
                                   fixed_over_time_frame_id_, rclcpp::Duration::from_seconds(1.0));

  std::vector<Eigen::Vector2d> grid_header_frame_path_points;
  grid_header_frame_path_points.reserve(base_path_points.size());
  for (const auto& point : base_path_points) {
    geometry_msgs::msg::PointStamped point_msg;
    geometry_msgs::msg::PointStamped transformed_point_msg;
    point_msg.header = target_header;
    point_msg.point.x = point.position.x();
    point_msg.point.y = point.position.y();
    point_msg.point.z = 0.0;
    tf2::doTransform(point_msg, transformed_point_msg, tf);
    grid_header_frame_path_points.emplace_back(transformed_point_msg.point.x, transformed_point_msg.point.y);
  }

  const double sample_step = static_cast<double>(grid_map_.info.resolution);
  const Eigen::Vector2d ego_center_offset(ego_data_.state.reference_point.translation_to_geometric_center.x,
                                          ego_data_.state.reference_point.translation_to_geometric_center.y);
  const Eigen::Vector2d grid_origin(grid_map_.info.origin.position.x, grid_map_.info.origin.position.y);
  const double grid_origin_yaw = tf2::getYaw(grid_map_.info.origin.orientation);
  const double grid_length_x = static_cast<double>(grid_map_.info.width) * sample_step;
  const double grid_length_y = static_cast<double>(grid_map_.info.height) * sample_step;

  // The target trajectory frame is ego-centered (normally base_link), so the ego reference point is its origin.
  const Eigen::Vector2d ego_reference_position = Eigen::Vector2d::Zero();
  double ego_path_s = 0.0;
  double best_dist_sq = std::numeric_limits<double>::max();
  for (size_t i = 0; i + 1 < base_path_points.size(); ++i) {
    const Eigen::Vector2d segment = base_path_points[i + 1].position - base_path_points[i].position;
    const double segment_length_sq = segment.squaredNorm();
    if (segment_length_sq < 1e-9) {
      continue;
    }

    const double alpha =
        std::clamp((ego_reference_position - base_path_points[i].position).dot(segment) / segment_length_sq, 0.0, 1.0);
    const Eigen::Vector2d projected = base_path_points[i].position + alpha * segment;
    const double dist_sq = (projected - ego_reference_position).squaredNorm();
    if (dist_sq >= best_dist_sq) {
      continue;
    }

    best_dist_sq = dist_sq;
    ego_path_s = base_path_points[i].s + alpha * (base_path_points[i + 1].s - base_path_points[i].s);
  }

  for (size_t i = 0; i + 1 < base_path_points.size(); ++i) {
    const Eigen::Vector2d segment = grid_header_frame_path_points[i + 1] - grid_header_frame_path_points[i];
    const double segment_length = segment.norm();
    if (segment_length <= 1e-2) {
      continue;
    }

    const Eigen::Vector2d tangent = segment / segment_length;
    const auto sample_count = static_cast<size_t>(std::ceil(segment_length / sample_step));
    for (size_t sample_idx = 0; sample_idx <= sample_count; ++sample_idx) {
      const double clamped_ds = std::min(static_cast<double>(sample_idx) * sample_step, segment_length);
      const double alpha = clamped_ds / segment_length;
      const double sample_s = base_path_points[i].s + alpha * (base_path_points[i + 1].s - base_path_points[i].s);
      if (sample_s < ego_path_s) {
        continue;
      }

      const Eigen::Vector2d reference_point = grid_header_frame_path_points[i] + alpha * segment;
      const double yaw = wrap_angle_rad(std::atan2(tangent.y(), tangent.x()));
      const Eigen::Vector2d ego_center = reference_point + rotate(ego_center_offset, yaw);
      const OrientedBox2D ego_box = buildOrientedBox(ego_center, yaw, ego_data_.length, ego_data_.width);

      const auto expanded_corners = getBoxCorners(ego_box, grid_longitudinal_safety_distance_, grid_lateral_safety_distance_);
      double min_local_x = std::numeric_limits<double>::max();
      double max_local_x = std::numeric_limits<double>::lowest();
      double min_local_y = std::numeric_limits<double>::max();
      double max_local_y = std::numeric_limits<double>::lowest();
      for (const auto& corner : expanded_corners) {
        const Eigen::Vector2d local_corner = transformToGridLocal(corner, grid_origin, grid_origin_yaw);
        min_local_x = std::min(min_local_x, local_corner.x());
        max_local_x = std::max(max_local_x, local_corner.x());
        min_local_y = std::min(min_local_y, local_corner.y());
        max_local_y = std::max(max_local_y, local_corner.y());
      }

      const bool box_out_of_grid =
          min_local_x < 0.0 || min_local_y < 0.0 || max_local_x >= grid_length_x || max_local_y >= grid_length_y;
      if (box_out_of_grid && consider_out_of_grid_) {
        return sample_s;
      }

      const int min_cell_x = std::max(static_cast<int>(std::floor(min_local_x / sample_step)), 0);
      const int max_cell_x =
          std::min(static_cast<int>(std::floor(max_local_x / sample_step)), static_cast<int>(grid_map_.info.width) - 1);
      const int min_cell_y = std::max(static_cast<int>(std::floor(min_local_y / sample_step)), 0);
      const int max_cell_y =
          std::min(static_cast<int>(std::floor(max_local_y / sample_step)), static_cast<int>(grid_map_.info.height) - 1);

      if (min_cell_x > max_cell_x || min_cell_y > max_cell_y) {
        continue;
      }

      for (int cell_y = min_cell_y; cell_y <= max_cell_y; ++cell_y) {
        for (int cell_x = min_cell_x; cell_x <= max_cell_x; ++cell_x) {
          const size_t index = static_cast<size_t>(cell_y) * grid_map_.info.width + static_cast<size_t>(cell_x);
          if (index >= grid_map_.data.size() || !isGridMapCellOccupied(grid_map_.data[index])) {
            continue;
          }

          const Eigen::Vector2d cell_center_local((static_cast<double>(cell_x) + 0.5) * sample_step,
                                                  (static_cast<double>(cell_y) + 0.5) * sample_step);
          const OrientedBox2D cell_box = buildOrientedBox(transformToGridFrame(cell_center_local, grid_origin, grid_origin_yaw),
                                                          grid_origin_yaw, sample_step, sample_step);
          if (overlapsWithEgoSafety(ego_box, cell_box, grid_longitudinal_safety_distance_, grid_lateral_safety_distance_)) {
            return sample_s;
          }
        }
      }
    }
  }

  return std::nullopt;
}

bool SimplePlannerNode::isGridMapCellOccupied(int8_t value) const { return value < 0 || value >= grid_occupied_threshold_; }

}  // namespace simple_planner
