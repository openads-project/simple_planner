#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <simple_planner/simple_planner_node.hpp>
#include <tf2_perception_msgs/tf2_perception_msgs.hpp>

// Object-handling part of SimplePlannerNode: turning the perceived object list into a speed cap
// on the route-following trajectory, plus the RViz visualization of the explaining conflict.
// The pure geometry helpers used here live in simple_planner/object_geometry.hpp.

namespace simple_planner {

void SimplePlannerNode::resetObjectState(const std_msgs::msg::Header& target_header) {
  last_object_speed_cap_.reset();
  object_conflict_free_cycles_ = 0;
  clearObjectInteractionMarkers(target_header);
}

void SimplePlannerNode::clearObjectInteractionMarkers(const std_msgs::msg::Header& target_header) {
  if (!object_interaction_marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  const std::array<std::string, 3> namespaces = {
      "object_interaction_ego_safety_box",
      "object_interaction_ego_box",
      "object_interaction_object_box"};
  for (size_t idx = 0; idx < namespaces.size(); ++idx) {
    visualization_msgs::msg::Marker marker;
    marker.header = target_header;
    marker.ns = namespaces[idx];
    marker.id = static_cast<int>(idx);
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.push_back(marker);
  }
  object_interaction_marker_pub_->publish(marker_array);
}

void SimplePlannerNode::publishObjectInteractionMarkers(const std_msgs::msg::Header& target_header,
                                                        const std::optional<ConflictSample>& conflict) {
  if (!object_interaction_marker_pub_) {
    return;
  }

  if (!publish_object_interaction_markers_ || !conflict.has_value()) {
    clearObjectInteractionMarkers(target_header);
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  auto make_box_marker = [&](const std::string& ns, int marker_id, const geometry_msgs::msg::Pose& pose,
                             double length, double width, double z, float r, float g, float b, float a) {
    visualization_msgs::msg::Marker marker;
    marker.header = target_header;
    marker.ns = ns;
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose;
    marker.pose.position.z = z;
    marker.scale.x = length;
    marker.scale.y = width;
    marker.scale.z = 0.08;
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 500000000;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker_array.markers.push_back(marker);
  };

  const geometry_msgs::msg::Pose ego_pose = toPose(conflict->ego_box);
  const double ego_length = 2.0 * conflict->ego_box.half_length;
  const double ego_width = 2.0 * conflict->ego_box.half_width;
  make_box_marker("object_interaction_ego_safety_box", 0, ego_pose,
                  ego_length + 2.0 * object_longitudinal_safety_distance_, ego_width + 2.0 * object_lateral_safety_distance_,
                  0.12, 1.0f, 0.55f, 0.0f, 0.28f);
  make_box_marker("object_interaction_ego_box", 1, ego_pose,
                  ego_length, ego_width, 0.18, 1.0f, 0.0f, 0.0f, 0.55f);
  make_box_marker("object_interaction_object_box", 2, toPose(conflict->object_box),
                  2.0 * conflict->object_box.half_length, 2.0 * conflict->object_box.half_width,
                  0.24, 0.0f, 0.45f, 1.0f, 0.55f);

  object_interaction_marker_pub_->publish(marker_array);
}

std::vector<ObjectTrajectory> SimplePlannerNode::buildObjectTrajectories(const perception_msgs::msg::ObjectList& tf_object_list,
                                                                         const rclcpp::Time& stamp) const {
  std::vector<ObjectTrajectory> object_trajectories;
  for (const auto& object : tf_object_list.objects) {
    double object_width = 0.0;
    double object_length = 0.0;
    try {
      object_width = perception_msgs::object_access::getWidth(object);
      object_length = perception_msgs::object_access::getLength(object);
    } catch (const std::exception&) {
      object_width = 0.0;
      object_length = 0.0;
    }
    if (!std::isfinite(object_width) || object_width < kMinObjectWidth) object_width = kMinObjectWidth;
    if (!std::isfinite(object_length) || object_length < kMinObjectLength) object_length = kMinObjectLength;

    auto add_static_object = [&]() {
      ObjectTrajectory trajectory;
      trajectory.id = object.id;
      trajectory.is_static = true;
      trajectory.samples.push_back(buildObjectSample(object.state, tf_object_list.header, stamp, object_length, object_width));
      object_trajectories.push_back(trajectory);
    };

    // Select all sufficiently likely predictions, or fall back to the single most likely one.
    std::vector<const perception_msgs::msg::ObjectStatePrediction*> selected_predictions;
    for (const auto& prediction : object.state_predictions) {
      if (prediction.probability >= min_prediction_prob_) selected_predictions.push_back(&prediction);
    }
    if (selected_predictions.empty() && !object.state_predictions.empty()) {
      selected_predictions.push_back(&*std::max_element(
          object.state_predictions.begin(), object.state_predictions.end(),
          [](const auto& lhs, const auto& rhs) { return lhs.probability < rhs.probability; }));
    }

    if (selected_predictions.empty()) {
      add_static_object();
      continue;
    }

    for (const auto* prediction : selected_predictions) {
      if (prediction == nullptr || prediction->states.empty()) {
        add_static_object();
        continue;
      }
      ObjectTrajectory trajectory;
      trajectory.id = object.id;
      trajectory.is_static = false;
      trajectory.samples.reserve(prediction->states.size());
      for (const auto& state : prediction->states) {
        trajectory.samples.push_back(buildObjectSample(state, tf_object_list.header, stamp, object_length, object_width));
      }
      object_trajectories.push_back(trajectory);
    }
  }
  return object_trajectories;
}

std::optional<ConflictSample> SimplePlannerNode::firstConflict(const std::vector<SimplePathPoint>& ego_path,
                                                               const std::vector<ObjectTrajectory>& object_trajectories) const {
  if (ego_path.empty()) return std::nullopt;

  const auto& ego_offset_msg = ego_data_.state.reference_point.translation_to_geometric_center;
  const Eigen::Vector2d ego_center_offset(ego_offset_msg.x, ego_offset_msg.y);

  // Interpolates the ego bounding box along the (time-equidistant) path at relative time ego_t.
  auto ego_box_at = [&](double ego_t) {
    const size_t idx = std::min(static_cast<size_t>(std::floor(std::max(ego_t, 0.0) / dt_)), ego_path.size() - 1);
    const size_t next_idx = std::min(idx + 1, ego_path.size() - 1);
    const double segment_start_t = dt_ * static_cast<double>(idx);
    const double alpha = next_idx > idx ? std::clamp((ego_t - segment_start_t) / dt_, 0.0, 1.0) : 0.0;
    const Eigen::Vector2d position = ego_path[idx].position + alpha * (ego_path[next_idx].position - ego_path[idx].position);
    Eigen::Vector2d heading = ego_path[next_idx].position - ego_path[idx].position;
    if (heading.squaredNorm() < 1e-9 && idx > 0) heading = ego_path[idx].position - ego_path[idx - 1].position;
    const double yaw = heading.squaredNorm() > 1e-9 ? wrap_angle_rad(std::atan2(heading.y(), heading.x())) : 0.0;
    return buildOrientedBox(position + rotate(ego_center_offset, yaw), yaw, ego_data_.length, ego_data_.width);
  };

  const double check_dt = std::clamp(kObjectCollisionCheckDt, 0.01, dt_);
  const size_t last_segment_idx = ego_path.size() > 1 ? ego_path.size() - 2 : 0;
  for (size_t i = 0; i <= last_segment_idx; ++i) {
    const double segment_start_t = dt_ * static_cast<double>(i);
    if (segment_start_t > trajectory_horizon_) break;
    const double segment_end_t = std::min(dt_ * static_cast<double>(i + 1), trajectory_horizon_);
    const int steps = std::max(1, static_cast<int>(std::ceil((segment_end_t - segment_start_t) / check_dt)));

    for (int step = 0; step <= steps; ++step) {
      const double ego_t = segment_start_t + (segment_end_t - segment_start_t) * static_cast<double>(step) / static_cast<double>(steps);
      const OrientedBox2D ego_box = ego_box_at(ego_t);

      for (const auto& object_trajectory : object_trajectories) {
        if (object_trajectory.samples.empty()) continue;

        if (object_trajectory.is_static) {
          const auto& object_sample = object_trajectory.samples.front();
          if (overlapsWithEgoSafety(ego_box, object_sample.box, object_longitudinal_safety_distance_, object_lateral_safety_distance_)) {
            return ConflictSample{object_trajectory.id, ego_box, object_sample.box};
          }
          continue;
        }

        for (size_t sample_idx = 0; sample_idx < object_trajectory.samples.size(); ++sample_idx) {
          const auto& object_sample = object_trajectory.samples[sample_idx];
          const TimedBox2D* next_sample = sample_idx + 1 < object_trajectory.samples.size() ? &object_trajectory.samples[sample_idx + 1] : nullptr;

          TimedBox2D timed_object_sample = object_sample;
          if (next_sample != nullptr && ego_t >= object_sample.t - object_interaction_time_window_ &&
              ego_t <= next_sample->t + object_interaction_time_window_) {
            if (ego_t >= object_sample.t && ego_t <= next_sample->t) {
              timed_object_sample = interpolateTimedBox(object_sample, *next_sample, ego_t);
            } else if (std::abs(ego_t - next_sample->t) < std::abs(ego_t - object_sample.t)) {
              timed_object_sample = *next_sample;
            }
          } else if (std::abs(ego_t - object_sample.t) > object_interaction_time_window_) {
            continue;
          }

          if (overlapsWithEgoSafety(ego_box, timed_object_sample.box, object_longitudinal_safety_distance_, object_lateral_safety_distance_)) {
            return ConflictSample{object_trajectory.id, ego_box, timed_object_sample.box};
          }
        }
      }
    }
  }
  return std::nullopt;
}

void SimplePlannerNode::applyObjectConstraints(const std_msgs::msg::Header& target_header,
                                               const std::vector<SimplePathPoint>& base_path_points,
                                               FollowRoutePlan& route_plan) {
  if (!consider_objects_ || !object_list_init_ || route_plan.path.points.empty() || base_path_points.empty()) {
    // Only forget the remembered speed cap if objects are disabled / never received; otherwise keep it
    // across cycles in which the route path happens to be empty.
    if (!consider_objects_ || !object_list_init_) {
      last_object_speed_cap_.reset();
      object_conflict_free_cycles_ = 0;
    }
    clearObjectInteractionMarkers(target_header);
    return;
  }

  const rclcpp::Time stamp(target_header.stamp);
  if (isMessageOutdated(object_list_.header, object_timeout_, stamp)) {
    std::string msg = "Object list is older than " + std::to_string(object_timeout_) + " seconds. Ignoring objects for this planning cycle.";
    setHealth(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg, health_.key_value_pairs);
    RCLCPP_DEBUG(this->get_logger(), "%s", msg.c_str());
    resetObjectState(target_header);
    return;
  }

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf2_buffer_->lookupTransform(target_header.frame_id, target_header.stamp, object_list_.header.frame_id, object_list_.header.stamp,
                                      fixed_over_time_frame_id_, rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException& ex) {
    std::string msg = "Object transformation is not available: " + std::string(ex.what()) + ". Ignoring objects for this planning cycle.";
    setHealth(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg, health_.key_value_pairs);
    RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
    resetObjectState(target_header);
    return;
  }

  perception_msgs::msg::ObjectList tf_object_list;
  tf2::doTransform(object_list_, tf_object_list, tf);
  // Ignore objects whose center is behind the ego vehicle (trajectory frame, +x ahead).
  tf_object_list.objects.erase(
      std::remove_if(tf_object_list.objects.begin(), tf_object_list.objects.end(), [](const auto& object) {
        return perception_msgs::object_access::getCenterPosition(object.state).x < 0.0;
      }),
      tf_object_list.objects.end());

  const std::vector<ObjectTrajectory> object_trajectories = buildObjectTrajectories(tf_object_list, stamp);
  if (object_trajectories.empty()) {
    resetObjectState(target_header);
    return;
  }

  double initial_speed_cap = 0.0;
  for (const auto& point : route_plan.path.points) initial_speed_cap = std::max(initial_speed_cap, point.v);
  if (v_ref_ >= 0.0) initial_speed_cap = std::max(initial_speed_cap, v_ref_);

  // Start the search at the remembered cap and allow a single release step upwards once enough
  // conflict-free cycles have passed (hysteresis to avoid oscillating between speeds).
  const rclcpp::Time iteration_begin = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  const double remembered_speed_cap = std::clamp(last_object_speed_cap_.value_or(initial_speed_cap), 0.0, initial_speed_cap);
  double search_start_speed_cap = remembered_speed_cap;
  bool attempted_release = false;
  if (last_object_speed_cap_.has_value() && search_start_speed_cap < initial_speed_cap &&
      object_conflict_free_cycles_ >= object_velocity_release_hysteresis_cycles_) {
    search_start_speed_cap = std::min(search_start_speed_cap + object_velocity_release_step_, initial_speed_cap);
    attempted_release = search_start_speed_cap > remembered_speed_cap + 1e-6;
  }

  // Step the speed cap down until the resampled path is conflict-free (or reaches zero).
  double speed_cap = search_start_speed_cap;
  size_t iteration_count = 0;
  std::optional<ConflictSample> last_conflict;
  while (speed_cap > 0.0) {
    ++iteration_count;
    std::vector<SimplePathPoint> candidate_path = resamplePath(base_path_points, route_plan.stop_at_end,
                                                               route_plan.offset_to_stop_line, &speed_cap);
    const std::optional<ConflictSample> conflict = firstConflict(candidate_path, object_trajectories);
    if (conflict.has_value()) {
      last_conflict = conflict;
      speed_cap = std::max(speed_cap - object_velocity_reduction_step_, 0.0);
      continue;
    }

    const rclcpp::Time iteration_end = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    publishObjectInteractionMarkers(target_header,
                                    speed_cap < initial_speed_cap ? last_conflict : std::optional<ConflictSample>{});
    RCLCPP_DEBUG(this->get_logger(), "Object velocity iteration took %f ms (%zu iterations)",
                 (iteration_end - iteration_begin).seconds() * 1e3, iteration_count);

    last_object_speed_cap_ = speed_cap < initial_speed_cap ? std::optional<double>(speed_cap) : std::nullopt;

    // Hold a reduced cap for a few stable cycles before allowing the next release step upwards.
    if (speed_cap >= initial_speed_cap - 1e-6 || speed_cap + 1e-6 < search_start_speed_cap || attempted_release) {
      object_conflict_free_cycles_ = 0;
    } else {
      object_conflict_free_cycles_ = std::min(object_conflict_free_cycles_ + 1, object_velocity_release_hysteresis_cycles_);
    }

    if (speed_cap <= object_standstill_speed_threshold_) {
      health_.key_value_pairs.insert_or_assign("ObjectSpeedCap", std::to_string(speed_cap));
      if (last_conflict.has_value()) {
        health_.key_value_pairs.insert_or_assign("ReasonToStop", "Object conflict");
        health_.key_value_pairs.insert_or_assign("ObjectConflictId", std::to_string(last_conflict->object_id));
      }
      route_plan.path.points.clear();
      RCLCPP_INFO(this->get_logger(), "Object avoidance speed cap %f m/s is below standstill threshold %f m/s. Publishing standstill.",
                  speed_cap, object_standstill_speed_threshold_);
      return;
    }

    route_plan.path.points = candidate_path;
    if (speed_cap < initial_speed_cap) {
      health_.key_value_pairs.insert_or_assign("ObjectSpeedCap", std::to_string(speed_cap));
      if (last_conflict.has_value()) {
        health_.key_value_pairs.insert_or_assign("ObjectConflictId", std::to_string(last_conflict->object_id));
      }
    }
    return;
  }

  // Speed cap reached zero while still conflicting: publish standstill.
  route_plan.path.points = resamplePath(base_path_points, route_plan.stop_at_end, route_plan.offset_to_stop_line, &speed_cap);
  const rclcpp::Time iteration_end = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  RCLCPP_DEBUG(this->get_logger(), "Object velocity iteration took %f ms (%zu iterations)",
               (iteration_end - iteration_begin).seconds() * 1e3, iteration_count);
  last_object_speed_cap_ = speed_cap;
  const std::optional<ConflictSample> standstill_conflict = firstConflict(route_plan.path.points, object_trajectories);
  if (standstill_conflict.has_value()) {
    health_.key_value_pairs.insert_or_assign("ObjectSpeedCap", std::to_string(speed_cap));
    health_.key_value_pairs.insert_or_assign("ReasonToStop", "Object conflict");
    health_.key_value_pairs.insert_or_assign("ObjectConflictId", std::to_string(standstill_conflict->object_id));
    last_conflict = standstill_conflict;
    object_conflict_free_cycles_ = 0;
    RCLCPP_INFO(this->get_logger(), "Reduced reference speed cap to 0.0 m/s; object %lu still conflicts at standstill",
                standstill_conflict->object_id);
  } else {
    object_conflict_free_cycles_ = std::min(object_conflict_free_cycles_ + 1, object_velocity_release_hysteresis_cycles_);
    health_.key_value_pairs.insert_or_assign("ObjectSpeedCap", std::to_string(speed_cap));
    if (last_conflict.has_value()) {
      health_.key_value_pairs.insert_or_assign("ReasonToStop", "Object conflict");
      health_.key_value_pairs.insert_or_assign("ObjectConflictId", std::to_string(last_conflict->object_id));
    }
  }
  publishObjectInteractionMarkers(target_header, last_conflict);
  route_plan.path.points.clear();
}

}  // namespace simple_planner
