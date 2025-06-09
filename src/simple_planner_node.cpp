#include <chrono>
#include <cmath>
#include <functional>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include <tk/spline.h>

#include <simple_planner/simple_planner_node.hpp>

/**
 * @brief Namespace for simple_planner package
 *
 */
namespace simple_planner {

/**
 * @brief Creates a SimplePlannerNode node
 *
 */
SimplePlannerNode::SimplePlannerNode() : Node("simple_planner_node") {
  this->declareAndLoadParameter("trajectory_frame_id", trajectory_frame_id_,
                                "Frame ID of published reference trajectory");
  this->declareAndLoadParameter("fixed_over_time_frame_id", fixed_over_time_frame_id_,
                                "Frame ID of frame that is fixed over time for finding temporal transforms");
  this->declareAndLoadParameter("frequency", freq_, "frequency of publishing trajectory");
  this->declareAndLoadParameter("route_timeout", route_timeout_, "Time after which a received route is considered invalid (s) (use -1 for no timeout)");
  this->declareAndLoadParameter("trajectory_horizon", trajectory_horizon_, "time horizon of the reference trajectory (s)");
  this->declareAndLoadParameter("n_states", n_states_, "number of states in the trajectory");
  this->declareAndLoadParameter("internal_route_update", internal_route_update_,
                                "true: the route is received once and then updated locally in this node (cutting off the traveled route, etc.); false: the route is received cyclically (it is updated externally).");
  this->declareAndLoadParameter("interpolation_type", interpolation_type_, "0: linear, 1: cubic spline",
                                true, false, false, (std::optional<uint8_t>)0, (std::optional<uint8_t>)1);
  this->declareAndLoadParameter("v_ref", v_ref_, "reference velocity (m/s); set for all states in the trajectory. Set to '-1.0' to use velocity from route.");
  this->declareAndLoadParameter("a_max_decel", a_max_decel_, "maximum deceleration (m/s^2) - must be < 0.0");
  this->declareAndLoadParameter("consider_traffic_lights", consider_traffic_lights_, "true: planner will consider traffic lights; false: planner will ignore traffic lights");
  this->declareAndLoadParameter("offset_to_stop_line", offset_to_stop_line_,
                                "additional distance to stop in front of a stop line (m) (default: 0.0 -> stops with "
                                "front of vehicle at stop line)");
  this->declareAndLoadParameter("consider_future_states", consider_future_states_,
                                "true: trajectory will consider forecast of traffic light states; false: trajectory will only consider current traffic light state");
  this->declareAndLoadParameter("lane_change_distance_factor", lane_change_distance_factor_,
                                "factor multiplied with the current velocity to determine the lane change distance (m)");
  this->declareAndLoadParameter("lane_change_min_distance_factor", lane_change_min_distance_factor_,
                                "factor multiplied with the vehicle length to determine the minimum lane change distance (m)");

  this->setup();
}

template <typename T>
void SimplePlannerNode::declareAndLoadParameter(const std::string& name,
                                                T& param,
                                                const std::string& description,
                                                const bool add_to_auto_reconfigurable_params,
                                                const bool is_required,
                                                const bool read_only,
                                                const std::optional<double>& from_value,
                                                const std::optional<double>& to_value,
                                                const std::optional<double>& step_value,
                                                const std::string& additional_constraints) {

  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = description;
  param_desc.additional_constraints = additional_constraints;
  param_desc.read_only = read_only;

  auto type = rclcpp::ParameterValue(param).get_type();

  if (from_value.has_value() && to_value.has_value()) {
    if constexpr(std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1);
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value())).set__step(step);
      param_desc.integer_range = {range};
    } else if constexpr(std::is_floating_point_v<T>) {
      rcl_interfaces::msg::FloatingPointRange range;
      T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1.0);
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value())).set__step(step);
      param_desc.floating_point_range = {range};
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter type of parameter '%s' does not support specifying a range", name.c_str());
    }
  }

  this->declare_parameter(name, type, param_desc);

  try {
    param = this->get_parameter(name).get_value<T>();
    std::stringstream ss;
    ss << "Loaded parameter '" << name << "': ";
    if constexpr(is_vector_v<T>) {
      ss << "[";
      for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "");
      ss << "]";
    } else {
      ss << param;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    if (is_required) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Missing required parameter '" << name << "', exiting");
      exit(EXIT_FAILURE);
    } else {
      std::stringstream ss;
      ss << "Missing parameter '" << name << "', using default value: ";
      if constexpr(is_vector_v<T>) {
        ss << "[";
        for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "");
        ss << "]";
      } else {
        ss << param;
      }
      RCLCPP_WARN_STREAM(this->get_logger(), ss.str());
      this->set_parameters({rclcpp::Parameter(name, rclcpp::ParameterValue(param))});
    }
  }

  if (add_to_auto_reconfigurable_params) {
    std::function<void(const rclcpp::Parameter&)> setter = [&param](const rclcpp::Parameter& p) {
      param = p.get_value<T>();
    };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}


rcl_interfaces::msg::SetParametersResult SimplePlannerNode::parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {

  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
        RCLCPP_INFO(this->get_logger(), "Reconfigured parameter '%s'", param.get_name().c_str());
        break;
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  return result;
}

/**
 * @brief Sets up subscribers, publishers, and more.
 *
 */
void SimplePlannerNode::setup() {
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // calculate dt
  dt_ = trajectory_horizon_ / (n_states_ - 1);

  // create a publisher for publishing output trajectory
  pub_ = this->create_publisher<trajectory_planning_msgs::msg::Trajectory>(kOutputTopic, 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", pub_->get_topic_name());

  // create a timer for repeatedly invoking a callback to publish messages
  publish_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / freq_),
                                           std::bind(&SimplePlannerNode::publishTimerCallback, this));
  RCLCPP_INFO(this->get_logger(), "Publishing trajectory at '%f' hz", freq_);

  // create subscriber for egoData
  sub_egoData_ = this->create_subscription<perception_msgs::msg::EgoData>(
      kEgoDataTopic, 10, std::bind(&SimplePlannerNode::egoDataCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_egoData_->get_topic_name());

  // create subscriber for route
  sub_route_ = this->create_subscription<route_planning_msgs::msg::Route>(
      kRouteTopic, 10, std::bind(&SimplePlannerNode::routeCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_route_->get_topic_name());

  // create a callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(
      std::bind(&SimplePlannerNode::parametersCallback, this, std::placeholders::_1));
}

/**
 * @brief This callback is invoked when the subscriber receives a new egoData message
 *
 * @param[in] msg   egoData
 */
void SimplePlannerNode::egoDataCallback(const perception_msgs::msg::EgoData::UniquePtr msg) {
  ego_data_ = *msg;

  if (!ego_data_init_) {
    ego_data_init_ = true;
    RCLCPP_INFO(this->get_logger(), "Received first ego data message, initialized global variable");
  }
}

/**
 * @brief This callback is invoked when the subscriber receives a new route message
 *
 * @param[in] msg   route
 */
void SimplePlannerNode::routeCallback(const route_planning_msgs::msg::Route::UniquePtr msg) {
  route_ = *msg;
  RCLCPP_INFO(this->get_logger(), "Received route message, initialized global variable");
  if (!route_init_) route_init_ = true;
}

/**
 * @brief Main function of this node. Creates a reference trajectory based on the current route, vehicle state, and planning parameters.
 *
 * This function generates a trajectory message by processing the current route, transforming it to the appropriate frame,
 * handling special cases (such as route timeouts or empty routes), and considering traffic lights and lane changes.
 * The resulting trajectory is resampled over time and trimmed to fit the configured number of states.
 *
 * @throws std::runtime_error if the route is too old.
 * @return trajectory_planning_msgs::msg::Trajectory The generated trajectory message.
 */
trajectory_planning_msgs::msg::Trajectory SimplePlannerNode::createTrajectory() {
  rclcpp::Time begin = rclcpp::Clock(RCL_SYSTEM_TIME).now();

  // define trajectory message and set header
  int type_id = trajectory_planning_msgs::REFERENCE::TYPE_ID;
  trajectory_planning_msgs::msg::Trajectory tra;
  tra.header.stamp = now();
  tra.header.frame_id = trajectory_frame_id_;

  // handle special cases
  if ((route_timeout_ != -1.0) && (rclcpp::Time(tra.header.stamp) - rclcpp::Time(route_.header.stamp)) > rclcpp::Duration::from_seconds(route_timeout_)) {
    route_init_ = false;
    throw std::runtime_error("Route is older than " + std::to_string(route_timeout_) + ".");
  } else if (route_.route_elements.empty()) {
    trajectory_planning_msgs::trajectory_access::initializeTrajectory(tra, type_id, 1);
    route_init_ = false;
    RCLCPP_WARN(this->get_logger(), "Route has no route_elements. Publishing standstill trajectory."); // TODO: just do nothing?
    return tra;
  }

  // time-transform route to current trajectory_frame_id_ frame
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf =
        tf2_buffer_->lookupTransform(tra.header.frame_id, tra.header.stamp, route_.header.frame_id, route_.header.stamp,
                                     fixed_over_time_frame_id_, rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Tranformation is not available: %s", ex.what());
  }
  route_planning_msgs::msg::Route tf_route;
  tf2::doTransform(route_, tf_route, tf);

  // convert route to simple path
  bool stop_at_end = false;
  double t_total = 0.0;
  double offset_to_stop_line = 0.0;
  std::vector<SimplePathPoint> path;
  std::map<uint64_t, uint64_t> lane_change_indices_map; // maps lane change idx (j) to start lane change idx (i_start)
  RCLCPP_INFO(this->get_logger(), "Number of remaining route elements: %zu", tf_route.destination_route_element_idx - tf_route.current_route_element_idx);
  for (size_t j = tf_route.current_route_element_idx; j < tf_route.destination_route_element_idx; ++j) {
    const auto& route_element = tf_route.route_elements[j];
    if (!route_element.is_enriched) {
      RCLCPP_WARN(this->get_logger(), "Route element %zu is not enriched. Skipping.", j);
      continue;
    }

    const auto& suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(route_element);
    SimplePathPoint simple_path_point;
    simple_path_point.position = Eigen::Vector2d(suggested_lane.reference_pose.position.x, suggested_lane.reference_pose.position.y);
    simple_path_point.s = route_element.s;
    simple_path_point.v = v_ref_; // default: use constant velocity from params
    if (v_ref_ < 0.0) { // use velocity from route if param v_ref_ is negative
      simple_path_point.v = suggested_lane.speed_limit / 3.6; // convert km/h to m/s
    }

    if (path.size() > 0) {
      // calculate time difference to previous point
      double v_average = (path.back().v + simple_path_point.v) / 2.0; // average velocity between last point and current point
      double dt = (simple_path_point.s - path.back().s) / v_average; // time difference to previous point
      if (dt < 0.0) {
        RCLCPP_WARN(this->get_logger(), "Negative time difference %f between points at s=%f and s=%f. Could lead to unexpected behavior.", dt, path.back().s, simple_path_point.s);
      }
      t_total += dt;
    }

    // get starting lane change index
    if (route_element.will_change_suggested_lane) {
      if (j+1 >= tf_route.route_elements.size()) {
        RCLCPP_WARN(this->get_logger(), "Route element %zu is the last element. Cannot change lane.", j);
        break;
      }
      int i_end = j + 1; // lane change should end at the next route element

      size_t current_lane_idx = route_element.suggested_lane_idx;
      int lane_change_direction = route_planning_msgs::route_access::getLaneChangeDirection(route_element, tf_route.route_elements[j+1]);
      double ego_velocity = perception_msgs::object_access::getVelocityMagnitude(ego_data_);
      double lane_change_distance = std::max(lane_change_min_distance_factor_ * ego_data_.length, lane_change_distance_factor_ * ego_velocity);
      RCLCPP_INFO(this->get_logger(), "Lane change direction: %d, lane change distance: %f", lane_change_direction, lane_change_distance);

      double ds = 0.0;
      int i_start = j;
      while (ds < lane_change_distance && i_start > 0) {
        if (auto result = route_planning_msgs::route_access::getPrecedingLaneElementIdx(current_lane_idx, tf_route.route_elements[i_start-1])) {
          current_lane_idx = *result;
        } else {
          RCLCPP_WARN(this->get_logger(), "No preceding lane element found for route element %u", i_start);
          break;
        }
        if (!route_planning_msgs::route_access::hasAdjacentLane(tf_route.route_elements[i_start-1], current_lane_idx, lane_change_direction)) {
          RCLCPP_WARN(this->get_logger(), "No adjacent lane found for route element %u", i_start-1);
          break;
        }
        ds = ds + std::abs(tf_route.route_elements[i_start].s - tf_route.route_elements[i_start-1].s);
        i_start--;
      }

      if (!tf_route.route_elements[i_start].is_enriched || !tf_route.route_elements[i_end].is_enriched) {
        RCLCPP_WARN(this->get_logger(), "Not enough enriched route elements (%u, %u) for lane change.", i_start, i_end);
        break;
      }

      lane_change_indices_map[j] = i_start;
    }

    // check for traffic light along the route element
    if (consider_traffic_lights_) {
      const auto& reg_elems = route_planning_msgs::route_access::getRegulatoryElementsOfLaneElement(suggested_lane, route_element.regulatory_elements);
      for (size_t k = 0; k < reg_elems.size(); ++k) {
        if (reg_elems[k].type != route_planning_msgs::msg::RegulatoryElement::TYPE_TRAFFIC_LIGHT) continue;
        if (reg_elems[k].meta_value == route_planning_msgs::msg::RegulatoryElement::META_VALUE_MOVEMENT_ALLOWED && !consider_future_states_) continue;
        offset_to_stop_line = offset_to_stop_line_ + ego_data_.length / 2.0 + ego_data_.state.reference_point.translation_to_geometric_center.x;
        double dt_offset_to_stop_line = offset_to_stop_line / simple_path_point.v; // time to stop in front of traffic light

        if (reg_elems[k].has_validity_stamp && consider_future_states_) {
          double validity_duration = rclcpp::Time(reg_elems[k].validity_stamp).seconds() - rclcpp::Time(route_.header.stamp).seconds();
          if (validity_duration < (t_total - dt_offset_to_stop_line)) {
            if (reg_elems[k].meta_value == route_planning_msgs::msg::RegulatoryElement::META_VALUE_MOVEMENT_ALLOWED) {
              stop_at_end = true; // change from green to red until stop point is reached
            } else {
              continue;           // change from red to green until stop point is reached
            }
          } else {
            if (reg_elems[k].meta_value == route_planning_msgs::msg::RegulatoryElement::META_VALUE_MOVEMENT_ALLOWED) {
              continue;           // green light, no stop required
            } else {
              stop_at_end = true; // red light, stop required
            }
          }
        } else {
          stop_at_end = true; // no validity stamp or no future states considered, stop required
        }
      }
    }

    // add simple path point to path
    path.push_back(simple_path_point);

    // break loop if stop at end (traffic light or end of route) or if total time exceeds 2x trajectory horizon
    if (j == tf_route.destination_route_element_idx - 1) stop_at_end = true;
    if (stop_at_end || t_total >= 2.0 * trajectory_horizon_) break;
  }

  // generate lane change segments and insert them into path
  size_t current = 0;
  std::vector<SimplePathPoint> merged_path;
  for (const auto& lane_change_indices : lane_change_indices_map) {
    uint64_t lane_change_idx_route = lane_change_indices.first;
    uint64_t start_idx_route = lane_change_indices.second;
    int start_idx = std::max(static_cast<int>(start_idx_route - tf_route.current_route_element_idx), 0);
    int end_idx = (lane_change_idx_route + 1) - tf_route.current_route_element_idx; // lane change should end at the next route element
    merged_path.insert(merged_path.end(), path.begin() + current, path.begin() + start_idx);
    std::vector<SimplePathPoint> lane_change_path = generateLaneChangePath(start_idx_route, lane_change_idx_route, tf_route);
    merged_path.insert(merged_path.end(), lane_change_path.begin(), lane_change_path.end());
    current = end_idx+1;
  }
  merged_path.insert(merged_path.end(), path.begin() + current, path.end()); // add remaining tail
  recalculateS(merged_path); // recalculate s values for merged path
  path = std::move(merged_path); // replace path with merged_path

  // resample path over time
  std::vector<SimplePathPoint> resampled_path = resamplePath(path, stop_at_end, offset_to_stop_line);

  // remove first point of path as long as it is behind the ego vehicle
  while (!resampled_path.empty() && resampled_path[0].position.x() < 0.0) {
    resampled_path.erase(resampled_path.begin());
  }

  // keep maximum the first n_states_ in resampled_path (and therefore in trajectory)
  if ((size_t) n_states_ < resampled_path.size()) {
    resampled_path.erase(resampled_path.begin() + n_states_, resampled_path.end());
  }

  // init trajectory and fill with resampled_path (route) and velocity (const from param) data
  trajectory_planning_msgs::trajectory_access::initializeTrajectory(tra, type_id, n_states_);
  if (!resampled_path.empty()) { // only fill trajectory if resampled_path is not empty
    for (int i = 0; i < n_states_; i++) {
      int idx = (size_t)i < resampled_path.size() ? i : resampled_path.size() - 1; // multiple points at end of resampled_path if n_states_ > resampled_path.size()
      trajectory_planning_msgs::trajectory_access::setT(tra, dt_ * i, i);
      trajectory_planning_msgs::trajectory_access::setX(tra, resampled_path[idx].position.x(), i);
      trajectory_planning_msgs::trajectory_access::setY(tra, resampled_path[idx].position.y(), i);
      trajectory_planning_msgs::trajectory_access::setV(tra, resampled_path[idx].v, i);
      RCLCPP_DEBUG(this->get_logger(), "Debug: i: %d,  t: %f,  x: %f,  y: %f,  v: %f, s: %f", i,
                  dt_ * i, resampled_path[i].position.x(), resampled_path[i].position.y(), resampled_path[i].v, resampled_path[i].s);
    }
  }
  trajectory_planning_msgs::trajectory_access::setStandstill(tra, resampled_path.empty());

  RCLCPP_DEBUG(this->get_logger(), "Standstill = %d", tra.standstill);

  rclcpp::Time end = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  RCLCPP_DEBUG(this->get_logger(), "Trajectory creation took %f ms", (end - begin).seconds() * 1e3);
  return tra;
}

std::vector<SimplePathPoint> SimplePlannerNode::generateLaneChangePath(const int start_idx, const int turn_idx,
                                                                       const route_planning_msgs::msg::Route& route) {
  std::vector<SimplePathPoint> lane_change_path;
  int end_idx = turn_idx + 1; // lane change should end at the next route element
  if (start_idx >= end_idx || start_idx < 0 || end_idx >= static_cast<int>(route.route_elements.size())) {
    RCLCPP_WARN(this->get_logger(), "Invalid lane change indices: %d, %d", start_idx, end_idx);
    return lane_change_path;
  }

  int lane_change_direction = route_planning_msgs::route_access::getLaneChangeDirection(route.route_elements[turn_idx], route.route_elements[turn_idx + 1]);

  // Interpolate between the two elements
  for (int i = start_idx; i <= end_idx; ++i) {
    if ((i - route.current_route_element_idx) < 0) continue;

    const auto& route_element = route.route_elements[i];
    const auto& suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(route_element);
    if (i > turn_idx) lane_change_direction = 0; // -> i == turn_idx + 1 == end_idx
    const auto& adjacent_lane = route_planning_msgs::route_access::getAdjacentLane(route_element, route.route_elements[i].suggested_lane_idx, lane_change_direction);
    Eigen::Vector2d suggested_lane_pos(suggested_lane.reference_pose.position.x, suggested_lane.reference_pose.position.y);
    Eigen::Vector2d adjacent_lane_pos(adjacent_lane.reference_pose.position.x, adjacent_lane.reference_pose.position.y);
    double alpha = 0.5 * (1.0 + std::cos(M_PI * static_cast<double>(i - start_idx) / (end_idx - start_idx)));
    Eigen::Vector2d interpolated_pos = alpha * suggested_lane_pos + (1.0 - alpha) * adjacent_lane_pos;
    lane_change_path.push_back(SimplePathPoint(interpolated_pos, route_element.s, suggested_lane.speed_limit / 3.6)); // convert km/h to m/s
  }

  return lane_change_path;
}

void SimplePlannerNode::recalculateS(std::vector<SimplePathPoint>& path) {
  if (path.empty()) return;

  path[0].s = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    double ds = (path[i].position - path[i - 1].position).norm();
    path[i].s = path[i - 1].s + ds;
  }
}

std::vector<SimplePathPoint> SimplePlannerNode::resamplePath(const std::vector<SimplePathPoint>& path, bool stop_at_end, double offset_to_stop_line) {
  rclcpp::Time begin = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  if (path.empty()) {
    RCLCPP_WARN(this->get_logger(), "Route is empty. No resampling possible.");
    return path;
  }

  std::vector<SimplePathPoint> resampled_path;
  double s = path[0].s;

  tk::spline x_spline, y_spline;
  if (interpolation_type_ == InterpolationType::SPLINE && path.size() > 2) {
    std::vector<double> s_vector, x_vector, y_vector;
    for (size_t j = 0; j < path.size(); ++j) {
      s_vector.push_back(path[j].s);
      x_vector.push_back(path[j].position.x());
      y_vector.push_back(path[j].position.y());
    }
    x_spline.set_points(s_vector, x_vector);
    y_spline.set_points(s_vector, y_vector);
  }

  while (s < path.back().s) {
    // find index of segment in route (not required if using linearInterpolation from utils)
    int idx = -1;
    for (size_t j = 0; j < path.size() - 1; ++j) {
      if (s >= path[j].s && s <= path[j+1].s) {
        idx = j;
        break;
      }
    }

    double v = v_ref_; // case 1: constant velocity from params
    if (v_ref_ < 0.0 && idx >= 0) {
      v = path[idx].v + (path[idx+1].v - path[idx].v) / (path[idx+1].s - path[idx].s) * (s - path[idx].s); // case 1: override v_ref from route
    }

    double distance_to_stop = -0.5 * std::pow(v, 2) / a_max_decel_ + offset_to_stop_line;
    distance_to_stop = std::max(distance_to_stop, 0.0);
    double brake_point = path.back().s - distance_to_stop;
    double ds = v * dt_; // case 1: constant velocity
    if (s + ds > brake_point && stop_at_end) {
      if (s < brake_point) { // special case: braking point is between two states
        double ds_1 = brake_point - s; // distance with constant velocity to braking point
        double dt_1 = ds_1 / v; // time with constant velocity to braking point
        double dt_2 = dt_ - dt_1; // remaining time with deceleration
        double ds_2 = std::max(0.5 * a_max_decel_ * std::pow(dt_2, 2) + v * dt_2, 0.0);
        ds = ds_1 + ds_2;
      } else {
        v = std::sqrt(std::max(std::pow(v, 2) + 2 * a_max_decel_ * (s - brake_point), 0.0)); // case 2: deceleration (v(s))
        ds = 0.5 * a_max_decel_ * std::pow(dt_, 2) + v * dt_; // case 2: deceleration
        if (ds < 0.0) ds = path.back().s - s; // only add rest of route instead of driving backwards
      }
    }

    // interpolate point at s
    SimplePathPoint simple_path_point;
    if (path.size() == 1) {
      simple_path_point.position = path[0].position;
    }
    else if (interpolation_type_ == InterpolationType::SPLINE && path.size() > 2) { // spline interpolation
      simple_path_point.position.x() = x_spline(s);
      simple_path_point.position.y() = y_spline(s);
    }
    else if ((interpolation_type_ == InterpolationType::SPLINE && path.size() <= 2) || interpolation_type_ == InterpolationType::LINEAR) { // linear interpolation // TODO: could be improved by using our linearInterpolation function -> no need for idx anymore
      simple_path_point.position.x() = path[idx].position.x() + (path[idx+1].position.x() - path[idx].position.x()) / (path[idx+1].s - path[idx].s) * (s - path[idx].s);
      simple_path_point.position.y() = path[idx].position.y() + (path[idx+1].position.y() - path[idx].position.y()) / (path[idx+1].s - path[idx].s) * (s - path[idx].s);
    }
    else { // unsupported interpolation type
      RCLCPP_ERROR(this->get_logger(), "Unsupported interpolation type value %d", interpolation_type_);
      throw std::runtime_error("Unsupported interpolation type value");
    }
    simple_path_point.s = s;
    simple_path_point.v = v;
    resampled_path.push_back(simple_path_point);

    // increment s and v for next iteration
    s = s + ds;
    if (s == path.back().s && v == 0.0) break; // stop at end of route
  }

  rclcpp::Time end = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  RCLCPP_DEBUG(this->get_logger(), "Resampling route took %f ms", (end - begin).seconds() * 1e3);

  return resampled_path;
}

/**
 * @brief This callback is invoked every period seconds by the timer
 *
 */
void SimplePlannerNode::publishTimerCallback() {
  // if route and ego data are not received, do nothing
  if (!route_init_ || !ego_data_init_) {
    return;
  }

  try {
    trajectory_planning_msgs::msg::Trajectory msg = createTrajectory();
    pub_->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "Published Trajectory!");
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(this->get_logger(), "Error while creating trajectory, do not publish trajectory: %s", e.what());
  }
}

}  // namespace simple_planner

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_planner::SimplePlannerNode>());
  rclcpp::shutdown();

  return 0;
}