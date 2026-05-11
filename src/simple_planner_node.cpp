#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <optional>
#include <sstream>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include <tk/spline.h>
#include <tracetools/tracetools.h>

#include <simple_planner/simple_planner_node.hpp>
#include <simple_planner/utils.hpp>
#include <tf2_perception_msgs/tf2_perception_msgs.hpp>

namespace {

struct OrientedBox2D {
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  Eigen::Vector2d axis_x = Eigen::Vector2d::UnitX();
  Eigen::Vector2d axis_y = Eigen::Vector2d::UnitY();
  double half_length = 0.0;
  double half_width = 0.0;
};

struct TimedBox2D {
  OrientedBox2D box;
  double t = 0.0;
  double yaw = 0.0;
  double length = 0.0;
  double width = 0.0;
};

double wrap_angle_rad(double angle_rad, double min_val = -M_PI, double max_val = M_PI) {
  double capped_angle_rad = angle_rad;
  while (capped_angle_rad > max_val) capped_angle_rad -= 2 * M_PI;
  while (capped_angle_rad < min_val) capped_angle_rad += 2 * M_PI;
  return capped_angle_rad;
}

Eigen::Vector2d rotate(const Eigen::Vector2d& vec, double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Eigen::Vector2d(c * vec.x() - s * vec.y(), s * vec.x() + c * vec.y());
}

double getStateRelativeTime(const perception_msgs::msg::ObjectState& state, const std_msgs::msg::Header& fallback_header,
                            const rclcpp::Time& planning_stamp) {
  const bool has_state_stamp = state.header.stamp.sec != 0 || state.header.stamp.nanosec != 0;
  const auto& source_header = has_state_stamp ? state.header : fallback_header;
  return (rclcpp::Time(source_header.stamp) - planning_stamp).seconds();
}

OrientedBox2D buildOrientedBox(const Eigen::Vector2d& center, double yaw, double length, double width) {
  OrientedBox2D box;
  box.center = center;
  box.axis_x = rotate(Eigen::Vector2d::UnitX(), yaw);
  box.axis_y = rotate(Eigen::Vector2d::UnitY(), yaw);
  box.half_length = std::max(length, 0.0) / 2.0;
  box.half_width = std::max(width, 0.0) / 2.0;
  return box;
}

geometry_msgs::msg::Point toPoint(const Eigen::Vector2d& point) {
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = 0.0;
  return msg;
}

geometry_msgs::msg::Pose toPose(const OrientedBox2D& box) {
  geometry_msgs::msg::Pose pose;
  pose.position = toPoint(box.center);
  const double yaw = std::atan2(box.axis_x.y(), box.axis_x.x());
  pose.orientation.z = std::sin(yaw / 2.0);
  pose.orientation.w = std::cos(yaw / 2.0);
  return pose;
}

TimedBox2D interpolateTimedBox(const TimedBox2D& lhs, const TimedBox2D& rhs, double t) {
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

bool overlapsWithEgoSafety(const OrientedBox2D& ego_box, const OrientedBox2D& object_box,
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

}  // namespace

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
  this->declareAndLoadParameter("ego_data_timeout", ego_data_timeout_, "Time after which a received ego vehicle data is considered invalid (s) (use -1 for no timeout)");
  this->declareAndLoadParameter("object_timeout", object_timeout_, "Time after which a received object list is considered invalid (s) (use -1 for no timeout)");
  this->declareAndLoadParameter("trajectory_horizon", trajectory_horizon_, "time horizon of the reference trajectory (s)");
  this->declareAndLoadParameter("n_states", n_states_, "number of states in the trajectory");
  this->declareAndLoadParameter("interpolation_type", interpolation_type_, "0: linear, 1: cubic spline",
                                true, false, false, (std::optional<uint8_t>)0, (std::optional<uint8_t>)1);
  this->declareAndLoadParameter("v_ref", v_ref_, "reference velocity (m/s); set for all states in the trajectory. Set to '-1.0' to use velocity from route.");
  this->declareAndLoadParameter("a_decel", a_decel_, "desired deceleration for braking at stop lines or end of route (m/s^2) - must be < 0.0");
  this->declareAndLoadParameter("a_max_decel", a_max_decel_, "maximum deceleration for safe-stop trajectories (m/s^2) - must be < 0.0 and <= a_decel");
  this->declareAndLoadParameter("trigger_turn_signals", trigger_turn_signals_,
                                "true: planner will trigger turn signal services; false: planner will not request turn signals");
  this->declareAndLoadParameter("consider_traffic_lights", consider_traffic_lights_, "true: planner will consider traffic lights; false: planner will ignore traffic lights");
  this->declareAndLoadParameter("offset_to_stop_line", offset_to_stop_line_,
                                "additional distance to stop in front of a stop line (m) (default: 0.0 -> stops with "
                                "front of vehicle at stop line)");
  this->declareAndLoadParameter("ignore_stop_line_threshold", ignore_stop_line_threshold_,
                                "a stop line will be ignored if the front of the vehicle has already passed the stop line by more than this threshold (m)");
  this->declareAndLoadParameter("consider_future_states", consider_future_states_,
                                "true: trajectory will consider forecast of traffic light states; false: trajectory will only consider current traffic light state");
  this->declareAndLoadParameter("consider_objects", consider_objects_,
                                "true: planner will consider perceived objects on the route; false: planner will ignore objects");
  this->declareAndLoadParameter("min_prediction_prob", min_prediction_prob_,
                                "minimum probability for considering an object prediction branch");
  this->declareAndLoadParameter("object_longitudinal_safety_distance", object_longitudinal_safety_distance_,
                                "longitudinal clearance around ego/object bounding boxes for conflict detection (m)",
                                true, false, false, 0.0, 20.0, 0.1);
  this->declareAndLoadParameter("object_lateral_safety_distance", object_lateral_safety_distance_,
                                "lateral clearance around ego/object bounding boxes for conflict detection (m)",
                                true, false, false, 0.0, 10.0, 0.1);
  this->declareAndLoadParameter("object_interaction_time_window", object_interaction_time_window_,
                                "maximum time offset for counting a spatial overlap as interaction (s)");
  this->declareAndLoadParameter("object_velocity_reduction_step", object_velocity_reduction_step_,
                                "velocity decrement per object-avoidance iteration (m/s)",
                                true, false, false, 1e-3, 40.0, 1e-3);
  this->declareAndLoadParameter("object_velocity_release_step", object_velocity_release_step_,
                                "maximum velocity increase per cycle after hysteresis cleared object conflicts (m/s)",
                                true, false, false, 1e-3, 40.0, 1e-3);
  this->declareAndLoadParameter("object_standstill_speed_threshold", object_standstill_speed_threshold_,
                                "publish standstill if object avoidance would require a lower speed cap (m/s)",
                                true, false, false, 0.0, 10.0, 1e-3);
  this->declareAndLoadParameter("object_velocity_release_hysteresis_cycles", object_velocity_release_hysteresis_cycles_,
                                "number of conflict-free cycles required before increasing the remembered object speed cap",
                                true, false, false, 0.0, 100.0, 1.0);
  this->declareAndLoadParameter("object_collision_check_dt", object_collision_check_dt_,
                                "maximum time step used for swept object collision checks (s)",
                                true, false, false, 0.01, 1.0, 0.01);
  this->declareAndLoadParameter("object_min_width", object_min_width_,
                                "minimum object width used if reported dimensions are missing or too small (m)",
                                true, false, false, 0.0, 10.0, 0.1);
  this->declareAndLoadParameter("object_min_length", object_min_length_,
                                "minimum object length used if reported dimensions are missing or too small (m)",
                                true, false, false, 0.0, 20.0, 0.1);
  this->declareAndLoadParameter("object_interaction_time_window_growth", object_interaction_time_window_growth_,
                                "additional interaction time window per second of prediction horizon (s/s)",
                                true, false, false, 0.0, 1.0, 0.01);
  this->declareAndLoadParameter("object_interaction_time_window_max", object_interaction_time_window_max_,
                                "maximum dynamic-object interaction time window (s)",
                                true, false, false, 0.0, 5.0, 0.01);
  this->declareAndLoadParameter("publish_object_interaction_markers", publish_object_interaction_markers_,
                                "publish RViz markers for the conflict explaining the final speed reduction");
  this->declareAndLoadParameter("lane_change_distance_factor", lane_change_distance_factor_,
                                "factor multiplied with the current velocity to determine the lane change distance (m)");
  this->declareAndLoadParameter("lane_change_min_distance_factor", lane_change_min_distance_factor_,
                                "factor multiplied with the vehicle length to determine the minimum lane change distance (m)");

  // check parameters
  if (a_decel_ >= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid parameter: a_decel must be < 0.0");
    exit(EXIT_FAILURE);
  }
  if (a_max_decel_ > a_decel_) {
    RCLCPP_ERROR(this->get_logger(), "Invalid parameter: a_max_decel must be < 0.0 and <= a_decel");
    exit(EXIT_FAILURE);
  }

  // diagnostics parameters
  this->declareAndLoadParameter("diagnostic_updater.topic_diagnostics.ego_data.min_frequency", ego_data_topic_diagnostic_config_.min_frequency, "Minimum frequency for incoming ego-data messages", false, true, true);
  this->declareAndLoadParameter("diagnostic_updater.topic_diagnostics.ego_data.max_frequency", ego_data_topic_diagnostic_config_.max_frequency, "Maximum frequency for incoming ego-data messages", false, true, true);
  this->declareAndLoadParameter("diagnostic_updater.topic_diagnostics.ego_data.min_acceptable_timestamp_delta", ego_data_topic_diagnostic_config_.min_acceptable_timestamp_delta, "Minimum acceptable timestamp delta for incoming ego-data messages", false, true, true);
  this->declareAndLoadParameter("diagnostic_updater.topic_diagnostics.ego_data.max_acceptable_timestamp_delta", ego_data_topic_diagnostic_config_.max_acceptable_timestamp_delta, "Maximum acceptable timestamp delta for incoming ego-data messages", false, true, true);
  this->declareAndLoadParameter("diagnostic_updater.topic_diagnostics.object_list.min_frequency", object_list_topic_diagnostic_config_.min_frequency, "Minimum frequency for incoming object-list messages", false, true, true);
  this->declareAndLoadParameter("diagnostic_updater.topic_diagnostics.object_list.max_frequency", object_list_topic_diagnostic_config_.max_frequency, "Maximum frequency for incoming object-list messages", false, true, true);
  this->declareAndLoadParameter("diagnostic_updater.topic_diagnostics.object_list.min_acceptable_timestamp_delta", object_list_topic_diagnostic_config_.min_acceptable_timestamp_delta, "Minimum acceptable timestamp delta for incoming object-list messages", false, true, true);
  this->declareAndLoadParameter("diagnostic_updater.topic_diagnostics.object_list.max_acceptable_timestamp_delta", object_list_topic_diagnostic_config_.max_acceptable_timestamp_delta, "Maximum acceptable timestamp delta for incoming object-list messages", false, true, true);
  this->declareAndLoadParameter("diagnostic_updater.topic_diagnostics.route.min_frequency", route_topic_diagnostic_config_.min_frequency, "Minimum frequency for incoming route messages", false, true, true);
  this->declareAndLoadParameter("diagnostic_updater.topic_diagnostics.route.max_frequency", route_topic_diagnostic_config_.max_frequency, "Maximum frequency for incoming route messages", false, true, true);
  this->declareAndLoadParameter("diagnostic_updater.topic_diagnostics.route.min_acceptable_timestamp_delta", route_topic_diagnostic_config_.min_acceptable_timestamp_delta, "Minimum acceptable timestamp delta for incoming route messages", false, true, true);
  this->declareAndLoadParameter("diagnostic_updater.topic_diagnostics.route.max_acceptable_timestamp_delta", route_topic_diagnostic_config_.max_acceptable_timestamp_delta, "Maximum acceptable timestamp delta for incoming route messages", false, true, true);
  this->declareAndLoadParameter("diagnostic_updater.diagnosed_publishers.trajectory.min_frequency", diagnosed_publisher_config_.min_frequency, "Minimum frequency for published trajectory messages", false, true, true);
  this->declareAndLoadParameter("diagnostic_updater.diagnosed_publishers.trajectory.max_frequency", diagnosed_publisher_config_.max_frequency, "Maximum frequency for published trajectory messages", false, true, true);
  this->declareAndLoadParameter("diagnostic_updater.diagnosed_publishers.trajectory.min_acceptable_timestamp_delta", diagnosed_publisher_config_.min_acceptable_timestamp_delta, "Minimum acceptable timestamp delta for published trajectory messages", false, true, true);
  this->declareAndLoadParameter("diagnostic_updater.diagnosed_publishers.trajectory.max_acceptable_timestamp_delta", diagnosed_publisher_config_.max_acceptable_timestamp_delta, "Maximum acceptable timestamp delta for published trajectory messages", false, true, true);

  this->setup();
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
  object_interaction_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(kObjectInteractionMarkerTopic, 10);
  RCLCPP_INFO(this->get_logger(), "Publishing object interaction markers to '%s'", object_interaction_marker_pub_->get_topic_name());

  // create a timer for repeatedly invoking a callback to publish messages
  publish_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / freq_),
                                           std::bind(&SimplePlannerNode::publishTimerCallback, this));
  RCLCPP_INFO(this->get_logger(), "Publishing trajectory at '%f' hz", freq_);

  // create subscriber for egoData
  sub_egoData_ = this->create_subscription<perception_msgs::msg::EgoData>(
      kEgoDataTopic, 10, std::bind(&SimplePlannerNode::egoDataCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_egoData_->get_topic_name());

  sub_object_list_ = this->create_subscription<perception_msgs::msg::ObjectList>(
      kObjectListTopic, 10, std::bind(&SimplePlannerNode::objectListCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_object_list_->get_topic_name());

  // create subscriber for route
  sub_route_ = this->create_subscription<route_planning_msgs::msg::Route>(
      kRouteTopic, 10, std::bind(&SimplePlannerNode::routeCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_route_->get_topic_name());

  // create service clients for turn indicators and hazard lights
  left_turn_indicator_service_client_ = this->create_client<std_srvs::srv::SetBool>(kLeftTurnIndicatorSrv);
  RCLCPP_INFO(this->get_logger(), "Prepared service client for '%s'", left_turn_indicator_service_client_->get_service_name());
  right_turn_indicator_service_client_ = this->create_client<std_srvs::srv::SetBool>(kRightTurnIndicatorSrv);
  RCLCPP_INFO(this->get_logger(), "Prepared service client for '%s'", right_turn_indicator_service_client_->get_service_name());
  hazard_lights_service_client_ = this->create_client<std_srvs::srv::SetBool>(kHazardLightsSrv);
  RCLCPP_INFO(this->get_logger(), "Prepared service client for '%s'", hazard_lights_service_client_->get_service_name());

  // create a callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(
      std::bind(&SimplePlannerNode::parametersCallback, this, std::placeholders::_1));

  // Annotate message links for tracing: Trajectory is published periodically based on subscriptions to egoData and route
  std::vector<const void *> link_subs = {
      static_cast<const void *>(sub_egoData_->get_subscription_handle().get()),
      static_cast<const void *>(sub_object_list_->get_subscription_handle().get()),
      static_cast<const void *>(sub_route_->get_subscription_handle().get())
  };
  std::vector<const void *> link_pubs = {
      static_cast<const void *>(pub_->get_publisher_handle().get())
  };
  TRACETOOLS_TRACEPOINT(message_link_periodic_async, link_subs.data(), link_subs.size(), link_pubs.data(), link_pubs.size());


  // setup diagnostic updater
  diagnostic_updater_.setHardwareID(this->get_name());
  diagnostic_updater_.add("Health", this, &SimplePlannerNode::health);

  const int ego_data_topic_diagnostic_frequency_window_size = std::ceil(5 / (diagnostic_updater_.getPeriod().seconds() * ego_data_topic_diagnostic_config_.min_frequency));
  ego_data_topic_diagnostic_ = std::make_unique<diagnostic_updater::TopicDiagnostic>(
    kEgoDataTopic,
    diagnostic_updater_,
    diagnostic_updater::FrequencyStatusParam(&ego_data_topic_diagnostic_config_.min_frequency, &ego_data_topic_diagnostic_config_.max_frequency, 0.0, ego_data_topic_diagnostic_frequency_window_size),
    diagnostic_updater::TimeStampStatusParam(ego_data_topic_diagnostic_config_.min_acceptable_timestamp_delta, ego_data_topic_diagnostic_config_.max_acceptable_timestamp_delta)
  );

  const int route_topic_diagnostic_frequency_window_size = std::ceil(5 / (diagnostic_updater_.getPeriod().seconds() * route_topic_diagnostic_config_.min_frequency));
  route_topic_diagnostic_ = std::make_unique<diagnostic_updater::TopicDiagnostic>(
    kRouteTopic,
    diagnostic_updater_,
    diagnostic_updater::FrequencyStatusParam(&route_topic_diagnostic_config_.min_frequency, &route_topic_diagnostic_config_.max_frequency, 0.0, route_topic_diagnostic_frequency_window_size),
    diagnostic_updater::TimeStampStatusParam(route_topic_diagnostic_config_.min_acceptable_timestamp_delta, route_topic_diagnostic_config_.max_acceptable_timestamp_delta)
  );

  if (consider_objects_) {
    const int object_list_topic_diagnostic_frequency_window_size = std::ceil(5 / (diagnostic_updater_.getPeriod().seconds() * object_list_topic_diagnostic_config_.min_frequency));
    object_list_topic_diagnostic_ = std::make_unique<diagnostic_updater::TopicDiagnostic>(
      kObjectListTopic,
      diagnostic_updater_,
      diagnostic_updater::FrequencyStatusParam(&object_list_topic_diagnostic_config_.min_frequency, &object_list_topic_diagnostic_config_.max_frequency, 0.0, object_list_topic_diagnostic_frequency_window_size),
      diagnostic_updater::TimeStampStatusParam(object_list_topic_diagnostic_config_.min_acceptable_timestamp_delta, object_list_topic_diagnostic_config_.max_acceptable_timestamp_delta)
    );
  }

  const int diagnosed_publisher_frequency_window_size = std::ceil(5 / (diagnostic_updater_.getPeriod().seconds() * diagnosed_publisher_config_.min_frequency));
  diagnosed_publisher_ = std::make_unique<diagnostic_updater::DiagnosedPublisher<trajectory_planning_msgs::msg::Trajectory>>(
    pub_,
    diagnostic_updater_,
    diagnostic_updater::FrequencyStatusParam(&diagnosed_publisher_config_.min_frequency, &diagnosed_publisher_config_.max_frequency, 0.0, diagnosed_publisher_frequency_window_size),
    diagnostic_updater::TimeStampStatusParam(diagnosed_publisher_config_.min_acceptable_timestamp_delta, diagnosed_publisher_config_.max_acceptable_timestamp_delta)
  );

}

void SimplePlannerNode::health(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  stat.summary(health_.status, health_.message);
  for (const auto& [key, value] : health_.key_value_pairs) {
    stat.add(key, value);
  }
}

void SimplePlannerNode::setHealth(const unsigned char status, const std::string& msg, const std::map<std::string, std::string>& key_value_pairs) {
  health_.status = status;
  health_.message = msg;
  health_.key_value_pairs = key_value_pairs;
}

/**
 * @brief This callback is invoked when the subscriber receives a new egoData message
 *
 * @param[in] msg   egoData
 */
void SimplePlannerNode::egoDataCallback(const perception_msgs::msg::EgoData::UniquePtr msg) {
  ego_data_topic_diagnostic_->tick(msg->header.stamp);
  ego_data_ = *msg;

  if (!ego_data_init_) {
    ego_data_init_ = true;
    RCLCPP_INFO(this->get_logger(), "Received first ego data message, initialized global variable");
  }
}

void SimplePlannerNode::objectListCallback(const perception_msgs::msg::ObjectList::UniquePtr msg) {
  if (object_list_topic_diagnostic_ != nullptr) {
    object_list_topic_diagnostic_->tick(msg->header.stamp);
  }
  object_list_ = *msg;

  if (!object_list_init_) {
    object_list_init_ = true;
    RCLCPP_INFO(this->get_logger(), "Received first object list message, initialized global variable");
  }
}

/**
 * @brief This callback is invoked when the subscriber receives a new route message
 *
 * @param[in] msg   route
 */
void SimplePlannerNode::routeCallback(const route_planning_msgs::msg::Route::UniquePtr msg) {
  route_topic_diagnostic_->tick(msg->header.stamp);
  route_ = *msg; 

  if (!route_init_) {
    RCLCPP_INFO(this->get_logger(), "Received new route message, initialized global variable");
    route_init_ = true;
    safe_stop_distance_.reset();
  }
}

std::string SimplePlannerNode::plannerStatetoString(const SimplePlannerNode::PlannerState& state) const {
  switch (state) {
    case PlannerState::NoPublish:
      return "NoPublish";
    case PlannerState::Standstill:
      return "Standstill";
    case PlannerState::SafeStop:
      return "SafeStop";
    case PlannerState::FollowRoute:
      return "FollowRoute";
    default:
      return "Unknown";
  }
}

std::string SimplePlannerNode::turnSignalToString(const uint8_t& turn_signal) const {
  switch (turn_signal) {
    case route_planning_msgs::msg::LaneElement::SUGGESTED_TURN_SIGNAL_NONE:
      return "None";
    case route_planning_msgs::msg::LaneElement::SUGGESTED_TURN_SIGNAL_LEFT:
      return "Left";
    case route_planning_msgs::msg::LaneElement::SUGGESTED_TURN_SIGNAL_RIGHT:
      return "Right";
    case route_planning_msgs::msg::LaneElement::SUGGESTED_TURN_SIGNAL_HAZARD:
      return "Hazard";
    default:
      return "Unknown";
  }
}

SimplePlannerNode::PlannerState SimplePlannerNode::determinePlannerState(const rclcpp::Time& stamp) {
  // ego data missing -> no publish
  if (!ego_data_init_) {
    setHealth(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No ego data received yet", { {"PlannerState", plannerStatetoString(PlannerState::NoPublish)} });
    return PlannerState::NoPublish;
  }

  // ego data outdated -> no publish
  if (isMessageOutdated(ego_data_.header, ego_data_timeout_, stamp)) {
    ego_data_init_ = false;
    std::string msg = "EgoData is older than " + std::to_string(ego_data_timeout_) + " seconds. Skip publishing until fresh ego data arrives.";
    RCLCPP_DEBUG(this->get_logger(), "%s", msg.c_str());
    setHealth(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg, { {"PlannerState", plannerStatetoString(PlannerState::NoPublish)} });
    return PlannerState::NoPublish;
  }

  // no route received and no ongoing safe stop -> no publish
  if (!route_init_ && !safe_stop_distance_.has_value()) {
    setHealth(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No route received and no ongoing safe stop", { {"PlannerState", plannerStatetoString(PlannerState::NoPublish)} });
    return PlannerState::NoPublish;
  }

  // route outdated and vehicle in standstill -> standstill
  if (route_init_ && isMessageOutdated(route_.header, route_timeout_, stamp)) {
    if (perception_msgs::object_access::getStandstill(ego_data_)) {
      route_init_ = false;
      safe_stop_distance_.reset();
      latest_path_.points.clear();
      std::string msg = "Route is older than " + std::to_string(route_timeout_) + " seconds and ego vehicle is stationary. Publishing standstill trajectory.";
      RCLCPP_DEBUG(this->get_logger(), "%s", msg.c_str());
      setHealth(diagnostic_msgs::msg::DiagnosticStatus::OK, msg, { {"PlannerState", plannerStatetoString(PlannerState::Standstill)} });
      return PlannerState::Standstill;
    }
    route_init_ = false;

    // route outdated and vehicle still moving -> safe stop
    std::string msg = "Route is older than " + std::to_string(route_timeout_) + " seconds but ego vehicle is still moving. Executing safe stop trajectory.";
    RCLCPP_DEBUG(this->get_logger(), "%s", msg.c_str());
    setHealth(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg, { {"PlannerState", plannerStatetoString(PlannerState::SafeStop)} });
    return PlannerState::SafeStop;
  }

  // route received, but empty -> standstill
  if (route_init_ && route_.route_elements.empty()) {
    route_init_ = false;
    safe_stop_distance_.reset();
    latest_path_.points.clear();
    std::string msg = "Received route has no route elements. Publishing standstill trajectory.";
    RCLCPP_DEBUG(this->get_logger(), "%s", msg.c_str());
    setHealth(diagnostic_msgs::msg::DiagnosticStatus::OK, msg, { {"PlannerState", plannerStatetoString(PlannerState::Standstill)} });
    return PlannerState::Standstill;
  }

  // no fresh route, but safe stop already started -> safe stop
  if (!route_init_ && safe_stop_distance_.has_value()) {
    if (perception_msgs::object_access::getStandstill(ego_data_)) {
      safe_stop_distance_.reset();
      latest_path_.points.clear();
      std::string msg = "Safe stop finished. Ego vehicle is considered stationary. Publishing standstill trajectory.";
      RCLCPP_DEBUG(this->get_logger(), "%s", msg.c_str());
      setHealth(diagnostic_msgs::msg::DiagnosticStatus::OK, msg, { {"PlannerState", plannerStatetoString(PlannerState::Standstill)} });
      return PlannerState::Standstill;
    }
    std::string msg = "No fresh route available, but safe stop already started. Executing safe stop trajectory.";
    RCLCPP_DEBUG(this->get_logger(), "%s", msg.c_str());
    setHealth(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg, { {"PlannerState", plannerStatetoString(PlannerState::SafeStop)} });
    return PlannerState::SafeStop;
  }

  // fresh ego data and valid route available -> follow route
  setHealth(diagnostic_msgs::msg::DiagnosticStatus::OK, "Input information up to date. Following route.", { {"PlannerState", plannerStatetoString(PlannerState::FollowRoute)} });
  return PlannerState::FollowRoute;
}

bool SimplePlannerNode::isMessageOutdated(const std_msgs::msg::Header& header, double timeout, const rclcpp::Time& stamp) const {
  if (timeout == -1.0) {
    return false;
  }
  return (stamp - rclcpp::Time(header.stamp)) > rclcpp::Duration::from_seconds(timeout);
}

/**
 * @brief Main function of this node. Creates a reference trajectory based on the current route, vehicle state, and planning parameters.
 *
 * This function generates a trajectory message by processing the current route, transforming it to the appropriate frame,
 * handling special cases (such as route timeouts or empty routes), and considering traffic lights and lane changes.
 * The resulting trajectory is resampled over time and trimmed to fit the configured number of states.
 *
 * @return trajectory_planning_msgs::msg::Trajectory The generated trajectory message.
 */
trajectory_planning_msgs::msg::Trajectory SimplePlannerNode::createTrajectory(PlannerState state, const rclcpp::Time& stamp) {
  rclcpp::Time begin = rclcpp::Clock(RCL_SYSTEM_TIME).now();

  trajectory_planning_msgs::msg::Trajectory tra;
  tra.header.stamp = stamp;
  tra.header.frame_id = trajectory_frame_id_;

  if (state != PlannerState::FollowRoute) {
    last_object_speed_cap_.reset();
    object_conflict_free_cycles_ = 0;
    clearObjectInteractionMarkers(tra.header);
  }

  switch (state) {
    case PlannerState::Standstill:
      tra = buildStandstillTrajectory(tra.header);
      break;
    case PlannerState::SafeStop:
      if (!safe_stop_distance_.has_value()) {
        tra = buildTrajectoryFromSimplePath(buildSafeStopPath(tra.header));
      } else {
        RCLCPP_DEBUG(this->get_logger(), "Executing safe stop.");
        tra = buildTrajectoryFromSimplePath(transformPath(latest_path_, tra.header));
      }
      break;
    case PlannerState::FollowRoute: {
      safe_stop_distance_.reset();
      FollowRoutePlan route_plan = buildRoutePlan(tra.header);
      tra = buildTrajectoryFromSimplePath(route_plan.path);
      break;
    }
    case PlannerState::NoPublish:
      throw std::runtime_error("createTrajectory called for non-publish state");
  }

  rclcpp::Time end = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  RCLCPP_DEBUG(this->get_logger(), "Trajectory creation took %f ms", (end - begin).seconds() * 1e3);
  return tra;
}

trajectory_planning_msgs::msg::Trajectory SimplePlannerNode::buildStandstillTrajectory(const std_msgs::msg::Header& target_header) {
  int type_id = trajectory_planning_msgs::REFERENCE::TYPE_ID;
  trajectory_planning_msgs::msg::Trajectory tra;
  trajectory_planning_msgs::trajectory_access::initializeTrajectory(tra, type_id, 1);
  tra.header = target_header;
  trajectory_planning_msgs::trajectory_access::setStandstill(tra, true);
  return tra;
}

trajectory_planning_msgs::msg::Trajectory SimplePlannerNode::buildTrajectoryFromSimplePath(const SimplePath& path) {
  SimplePath usable_path = path;
  trimPathBehindEgo(usable_path);

  if (usable_path.points.empty()) {
    safe_stop_distance_.reset();
    latest_path_.points.clear();
    std::string msg = "No usable forward path remains. Publishing standstill trajectory.";
    RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
    health_.key_value_pairs.insert_or_assign("PlannerState", plannerStatetoString(PlannerState::Standstill));
    setHealth(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg, health_.key_value_pairs);
    return buildStandstillTrajectory(path.header);
  }

  latest_path_ = usable_path;
  std::vector<SimplePathPoint> path_points = latest_path_.points;
  if ((size_t) n_states_ < path_points.size()) {
    path_points.erase(path_points.begin() + n_states_, path_points.end());
  }

  int type_id = trajectory_planning_msgs::REFERENCE::TYPE_ID;
  trajectory_planning_msgs::msg::Trajectory tra;
  trajectory_planning_msgs::trajectory_access::initializeTrajectory(tra, type_id, n_states_);
  tra.header = usable_path.header;

  for (int i = 0; i < n_states_; i++) {
    int idx = (size_t)i < path_points.size() ? i : path_points.size() - 1;
    trajectory_planning_msgs::trajectory_access::setT(tra, dt_ * i, i);
    trajectory_planning_msgs::trajectory_access::setX(tra, path_points[idx].position.x(), i);
    trajectory_planning_msgs::trajectory_access::setY(tra, path_points[idx].position.y(), i);
    trajectory_planning_msgs::trajectory_access::setV(tra, path_points[idx].v, i);
    RCLCPP_DEBUG(this->get_logger(), "Debug: i: %d,  t: %f,  x: %f,  y: %f,  v: %f, s: %f", i,
                dt_ * i, path_points[idx].position.x(), path_points[idx].position.y(), path_points[idx].v, path_points[idx].s);
  }

  trajectory_planning_msgs::trajectory_access::setStandstill(tra, false);
  RCLCPP_DEBUG(this->get_logger(), "Standstill = %d", tra.standstill);
  return tra;
}

SimplePath SimplePlannerNode::buildSafeStopPath(const std_msgs::msg::Header& target_header) {
  double current_velocity = perception_msgs::object_access::getVelocityMagnitude(ego_data_);
  safe_stop_distance_ = -0.5 * std::pow(current_velocity, 2) / a_max_decel_;

  SimplePath safe_stop_path = transformPath(latest_path_, target_header);
  trimPathBehindEgo(safe_stop_path);
  if (safe_stop_path.points.empty()) {
    RCLCPP_WARN(this->get_logger(), "No latest path available. Initialize safe stop along ego heading. Current velocity: %f m/s, safe stop distance: %f m", current_velocity, *safe_stop_distance_);
    latest_path_ = transformPath(calculateSafeStopAlongEgoHeading(ego_data_, *safe_stop_distance_, target_header), target_header);
  } else {
    RCLCPP_WARN(this->get_logger(), "Initialize safe stop along latest path. Current velocity: %f m/s, safe stop distance: %f m", current_velocity, *safe_stop_distance_);
    latest_path_ = calculateSafeStopAlongRoute(safe_stop_path, *safe_stop_distance_);
  }
  return latest_path_;
}

SimplePlannerNode::FollowRoutePlan SimplePlannerNode::buildRoutePlan(const std_msgs::msg::Header& target_header) {
  RCLCPP_DEBUG(this->get_logger(), "Default case: route is up to date, creating path from route.");

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf2_buffer_->lookupTransform(target_header.frame_id, target_header.stamp, route_.header.frame_id, route_.header.stamp,
                                      fixed_over_time_frame_id_, rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException& ex) {
    std::string msg = "Transformation is not available: " + std::string(ex.what()) + ".";
    setHealth(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg, health_.key_value_pairs);
    RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
  }

  route_planning_msgs::msg::Route tf_route;
  tf2::doTransform(route_, tf_route, tf);
  FollowRoutePlan route_plan;
  route_plan.path.header = tf_route.header;

  std::map<uint64_t, uint64_t> lane_change_indices_map;
  appendRoutePoints(tf_route, route_plan, lane_change_indices_map);

  std::vector<SimplePathPoint> merged_points = mergeLaneChangeSegments(tf_route, route_plan.path.points, lane_change_indices_map);
  recalculateS(merged_points);
  route_plan.path.points = resamplePath(merged_points, route_plan.stop_at_end, route_plan.offset_to_stop_line);
  applyObjectConstraints(target_header, merged_points, route_plan);
  if (trigger_turn_signals_) {
    applyIndicatorRequest(route_plan.suggested_turn_signal);
    health_.key_value_pairs.insert_or_assign("SuggestedTurnSignal", turnSignalToString(route_plan.suggested_turn_signal));
  }
  if (route_plan.stop_at_end) health_.key_value_pairs.insert({"ReasonToStop", route_plan.reason_to_stop});
  return route_plan;
}

void SimplePlannerNode::applyObjectConstraints(const std_msgs::msg::Header& target_header,
                                               const std::vector<SimplePathPoint>& base_path_points,
                                               FollowRoutePlan& route_plan) {
  if (!consider_objects_ || route_plan.path.points.empty() || base_path_points.empty()) {
    if (!consider_objects_) {
      last_object_speed_cap_.reset();
      object_conflict_free_cycles_ = 0;
    }
    clearObjectInteractionMarkers(target_header);
    return;
  }

  if (!object_list_init_) {
    std::string msg = "No object list received yet. Ignoring objects for this planning cycle.";
    setHealth(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg, health_.key_value_pairs);
    last_object_speed_cap_.reset();
    object_conflict_free_cycles_ = 0;
    clearObjectInteractionMarkers(target_header);
    return;
  }

  const rclcpp::Time stamp(target_header.stamp);

  if (isMessageOutdated(object_list_.header, object_timeout_, stamp)) {
    std::string msg = "Object list is older than " + std::to_string(object_timeout_) + " seconds. Ignoring objects for this planning cycle.";
    setHealth(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg, health_.key_value_pairs);
    RCLCPP_DEBUG(this->get_logger(), "%s", msg.c_str());
    last_object_speed_cap_.reset();
    object_conflict_free_cycles_ = 0;
    clearObjectInteractionMarkers(target_header);
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
    last_object_speed_cap_.reset();
    object_conflict_free_cycles_ = 0;
    clearObjectInteractionMarkers(target_header);
    return;
  }

  perception_msgs::msg::ObjectList tf_object_list;
  tf2::doTransform(object_list_, tf_object_list, tf);
  tf_object_list.objects.erase(
      std::remove_if(tf_object_list.objects.begin(), tf_object_list.objects.end(), [](const auto& object) {
        return perception_msgs::object_access::getCenterPosition(object.state).x < 0.0;
      }),
      tf_object_list.objects.end());

  struct ObjectTrajectory {
    uint64_t id = 0;
    bool is_static = false;
    std::vector<TimedBox2D> samples;
  };

  const auto dynamic_time_window = [&](double t) {
    const double grown_window = object_interaction_time_window_ +
                                object_interaction_time_window_growth_ * std::max(t, 0.0);
    const double max_window = std::max(object_interaction_time_window_, object_interaction_time_window_max_);
    return std::clamp(grown_window, object_interaction_time_window_, max_window);
  };

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
    if (!std::isfinite(object_width) || object_width < object_min_width_) {
      object_width = object_min_width_;
    }
    if (!std::isfinite(object_length) || object_length < object_min_length_) {
      object_length = object_min_length_;
    }

    auto build_object_sample = [&](const perception_msgs::msg::ObjectState& state,
                                   const std_msgs::msg::Header& fallback_header) {
      const auto center_msg = perception_msgs::object_access::getCenterPosition(state);
      const Eigen::Vector2d center(center_msg.x, center_msg.y);
      TimedBox2D sample;
      sample.yaw = perception_msgs::object_access::getYaw(state);
      sample.length = object_length;
      sample.width = object_width;
      sample.box = buildOrientedBox(center, sample.yaw, sample.length, sample.width);
      sample.t = getStateRelativeTime(state, fallback_header, stamp);
      return sample;
    };

    auto add_static_object = [&]() {
      ObjectTrajectory trajectory;
      trajectory.id = object.id;
      trajectory.is_static = true;
      trajectory.samples.push_back(build_object_sample(object.state, tf_object_list.header));
      object_trajectories.push_back(trajectory);
    };

    std::vector<const perception_msgs::msg::ObjectStatePrediction*> selected_predictions;
    for (const auto& prediction : object.state_predictions) {
      if (prediction.probability >= min_prediction_prob_) {
        selected_predictions.push_back(&prediction);
      }
    }

    if (selected_predictions.empty() && !object.state_predictions.empty()) {
      const auto prediction_it = std::max_element(
          object.state_predictions.begin(), object.state_predictions.end(),
          [](const auto& lhs, const auto& rhs) { return lhs.probability < rhs.probability; });
      if (prediction_it != object.state_predictions.end()) {
        selected_predictions.push_back(&(*prediction_it));
      }
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
      for (size_t state_idx = 0; state_idx < prediction->states.size(); ++state_idx) {
        (void)state_idx;
        trajectory.samples.push_back(build_object_sample(prediction->states[state_idx], tf_object_list.header));
      }
      object_trajectories.push_back(trajectory);
    }
  }

  if (object_trajectories.empty()) {
    last_object_speed_cap_.reset();
    object_conflict_free_cycles_ = 0;
    clearObjectInteractionMarkers(target_header);
    return;
  }

  const auto& ego_offset_msg = ego_data_.state.reference_point.translation_to_geometric_center;
  const Eigen::Vector2d ego_center_offset(ego_offset_msg.x, ego_offset_msg.y);
  struct ConflictSample {
    uint64_t object_id = 0;
    double t = 0.0;
    OrientedBox2D ego_box;
    OrientedBox2D object_box;
  };

  auto build_ego_sample = [&](const std::vector<SimplePathPoint>& ego_path, double ego_t,
                              OrientedBox2D& ego_box) -> bool {
    if (ego_path.empty()) {
      return false;
    }

    size_t idx = std::min(static_cast<size_t>(std::floor(std::max(ego_t, 0.0) / dt_)), ego_path.size() - 1);
    size_t next_idx = std::min(idx + 1, ego_path.size() - 1);
    const double segment_start_t = dt_ * static_cast<double>(idx);
    const double alpha = next_idx > idx ? std::clamp((ego_t - segment_start_t) / dt_, 0.0, 1.0) : 0.0;
    const Eigen::Vector2d position = ego_path[idx].position + alpha * (ego_path[next_idx].position - ego_path[idx].position);

    Eigen::Vector2d heading = ego_path[next_idx].position - ego_path[idx].position;
    if (heading.squaredNorm() < 1e-9 && idx > 0) {
      heading = ego_path[idx].position - ego_path[idx - 1].position;
    }
    const double yaw = heading.squaredNorm() > 1e-9 ? wrap_angle_rad(std::atan2(heading.y(), heading.x())) : 0.0;
    const Eigen::Vector2d center = position + rotate(ego_center_offset, yaw);
    ego_box = buildOrientedBox(center, yaw, ego_data_.length, ego_data_.width);
    return true;
  };

  auto collect_current_conflicts = [&](const std::vector<SimplePathPoint>& ego_path) -> std::vector<ConflictSample> {
    std::vector<ConflictSample> conflicts;
    if (ego_path.empty()) {
      return conflicts;
    }

    const double check_dt = std::clamp(object_collision_check_dt_, 0.01, dt_);
    const size_t last_segment_idx = ego_path.size() > 1 ? ego_path.size() - 2 : 0;
    for (size_t i = 0; i <= last_segment_idx; ++i) {
      const double segment_start_t = dt_ * static_cast<double>(i);
      const double segment_end_t = std::min(dt_ * static_cast<double>(i + 1), trajectory_horizon_);
      if (segment_start_t > trajectory_horizon_) break;
      const int steps = std::max(1, static_cast<int>(std::ceil((segment_end_t - segment_start_t) / check_dt)));

      for (int step = 0; step <= steps; ++step) {
        const double ego_t = segment_start_t + (segment_end_t - segment_start_t) * static_cast<double>(step) / static_cast<double>(steps);
        OrientedBox2D ego_box;
        if (!build_ego_sample(ego_path, ego_t, ego_box)) {
          continue;
        }

        for (const auto& object_trajectory : object_trajectories) {
          if (object_trajectory.samples.empty()) {
            continue;
          }

          if (object_trajectory.is_static) {
            const auto& object_sample = object_trajectory.samples.front();
            if (!overlapsWithEgoSafety(ego_box, object_sample.box, object_longitudinal_safety_distance_,
                                       object_lateral_safety_distance_)) {
              continue;
            }
            conflicts.push_back({object_trajectory.id, ego_t, ego_box, object_sample.box});
            continue;
          }

          for (size_t sample_idx = 0; sample_idx < object_trajectory.samples.size(); ++sample_idx) {
            const auto& object_sample = object_trajectory.samples[sample_idx];
            const TimedBox2D* next_sample = sample_idx + 1 < object_trajectory.samples.size() ? &object_trajectory.samples[sample_idx + 1] : nullptr;
            const double time_window = dynamic_time_window(ego_t);

            TimedBox2D timed_object_sample = object_sample;
            if (next_sample != nullptr && ego_t >= object_sample.t - time_window && ego_t <= next_sample->t + time_window) {
              if (ego_t >= object_sample.t && ego_t <= next_sample->t) {
                timed_object_sample = interpolateTimedBox(object_sample, *next_sample, ego_t);
              } else if (std::abs(ego_t - next_sample->t) < std::abs(ego_t - object_sample.t)) {
                timed_object_sample = *next_sample;
              }
            } else if (std::abs(ego_t - object_sample.t) > time_window) {
              continue;
            }

            if (!overlapsWithEgoSafety(ego_box, timed_object_sample.box, object_longitudinal_safety_distance_,
                                       object_lateral_safety_distance_)) {
              continue;
            }
            conflicts.push_back({object_trajectory.id, ego_t, ego_box, timed_object_sample.box});
          }
        }
      }
    }

    return conflicts;
  };

  auto build_debug_sample = [&](const ConflictSample& conflict, double debug_speed_cap, size_t iteration) {
    ObjectConflictDebugSample sample;
    sample.object_id = conflict.object_id;
    sample.iteration = iteration;
    sample.speed_cap = debug_speed_cap;
    sample.t = conflict.t;
    sample.ego_pose = toPose(conflict.ego_box);
    sample.object_pose = toPose(conflict.object_box);
    sample.ego_length = 2.0 * conflict.ego_box.half_length;
    sample.ego_width = 2.0 * conflict.ego_box.half_width;
    sample.ego_safety_length = sample.ego_length + 2.0 * object_longitudinal_safety_distance_;
    sample.ego_safety_width = sample.ego_width + 2.0 * object_lateral_safety_distance_;
    sample.object_length = 2.0 * conflict.object_box.half_length;
    sample.object_width = 2.0 * conflict.object_box.half_width;
    return sample;
  };

  double initial_speed_cap = 0.0;
  for (const auto& point : route_plan.path.points) {
    initial_speed_cap = std::max(initial_speed_cap, point.v);
  }
  if (v_ref_ >= 0.0) {
    initial_speed_cap = std::max(initial_speed_cap, v_ref_);
  }

  const rclcpp::Time iteration_begin = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  const double remembered_speed_cap =
      std::clamp(last_object_speed_cap_.value_or(initial_speed_cap), 0.0, initial_speed_cap);
  double search_start_speed_cap = remembered_speed_cap;
  bool attempted_release = false;
  if (last_object_speed_cap_.has_value() && search_start_speed_cap < initial_speed_cap &&
      object_conflict_free_cycles_ >= object_velocity_release_hysteresis_cycles_) {
    search_start_speed_cap = std::min(search_start_speed_cap + object_velocity_release_step_, initial_speed_cap);
    attempted_release = search_start_speed_cap > remembered_speed_cap + 1e-6;
  }

  double speed_cap = search_start_speed_cap;
  size_t iteration_count = 0;
  std::optional<ObjectConflictDebugSample> conflict_debug_sample;
  while (speed_cap > 0.0) {
    ++iteration_count;
    std::vector<SimplePathPoint> candidate_path = resamplePath(base_path_points, route_plan.stop_at_end,
                                                               route_plan.offset_to_stop_line, &speed_cap);
    const std::vector<ConflictSample> current_candidate_conflicts = collect_current_conflicts(candidate_path);
    std::vector<ConflictSample> conflicts = current_candidate_conflicts;
    if (!conflicts.empty()) {
      conflict_debug_sample = build_debug_sample(conflicts.front(), speed_cap, iteration_count);
    }

    if (conflicts.empty()) {
      const rclcpp::Time iteration_end = rclcpp::Clock(RCL_SYSTEM_TIME).now();
      publishObjectInteractionMarkers(target_header,
                                      speed_cap < initial_speed_cap ? conflict_debug_sample : std::optional<ObjectConflictDebugSample>{});
      RCLCPP_INFO(this->get_logger(), "Object velocity iteration took %f ms (%zu iterations)",
                  (iteration_end - iteration_begin).seconds() * 1e3, iteration_count);

      if (speed_cap < initial_speed_cap) {
        last_object_speed_cap_ = speed_cap;
      } else {
        last_object_speed_cap_.reset();
      }

      // Hold a reduced cap for a few stable cycles before allowing the next release step upwards.
      if (speed_cap >= initial_speed_cap - 1e-6) {
        object_conflict_free_cycles_ = 0;
      } else if (speed_cap + 1e-6 < search_start_speed_cap || attempted_release) {
        object_conflict_free_cycles_ = 0;
      } else {
        object_conflict_free_cycles_ = std::min(object_conflict_free_cycles_ + 1, object_velocity_release_hysteresis_cycles_);
      }

      if (speed_cap <= object_standstill_speed_threshold_) {
        health_.key_value_pairs.insert_or_assign("ObjectSpeedCap", std::to_string(speed_cap));
        health_.key_value_pairs.insert_or_assign("ReasonToStop", "Object conflict");
        route_plan.path.points.clear();
        RCLCPP_INFO(this->get_logger(), "Object avoidance speed cap %f m/s is below standstill threshold %f m/s. Publishing standstill.",
                    speed_cap, object_standstill_speed_threshold_);
        return;
      }

      route_plan.path.points = candidate_path;
      if (speed_cap < initial_speed_cap) {
        health_.key_value_pairs.insert_or_assign("ObjectSpeedCap", std::to_string(speed_cap));
        health_.key_value_pairs.insert_or_assign("ObjectConflictId", std::to_string(conflict_debug_sample->object_id));
        RCLCPP_INFO(this->get_logger(), "Reduced reference speed cap to %f m/s to avoid object conflict", speed_cap);
      }
      return;
    }

    speed_cap = std::max(speed_cap - object_velocity_reduction_step_, 0.0);
  }

  route_plan.path.points = resamplePath(base_path_points, route_plan.stop_at_end, route_plan.offset_to_stop_line, &speed_cap);
  const rclcpp::Time iteration_end = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  RCLCPP_INFO(this->get_logger(), "Object velocity iteration took %f ms (%zu iterations)",
              (iteration_end - iteration_begin).seconds() * 1e3, iteration_count);
  last_object_speed_cap_ = speed_cap;
  const std::vector<ConflictSample> current_conflicts_at_standstill = collect_current_conflicts(route_plan.path.points);
  if (!current_conflicts_at_standstill.empty()) {
    conflict_debug_sample = build_debug_sample(current_conflicts_at_standstill.front(), speed_cap, iteration_count);
  }
  publishObjectInteractionMarkers(target_header, conflict_debug_sample);
  if (current_conflicts_at_standstill.empty()) {
    health_.key_value_pairs.insert_or_assign("ObjectSpeedCap", std::to_string(speed_cap));
    health_.key_value_pairs.insert_or_assign("ReasonToStop", "Object conflict");
    object_conflict_free_cycles_ = std::min(object_conflict_free_cycles_ + 1, object_velocity_release_hysteresis_cycles_);
    RCLCPP_INFO(this->get_logger(), "Reduced reference speed cap to 0.0 m/s to avoid object conflict");
  } else {
    health_.key_value_pairs.insert_or_assign("ObjectSpeedCap", std::to_string(speed_cap));
    health_.key_value_pairs.insert_or_assign("ReasonToStop", "Object conflict");
    health_.key_value_pairs.insert_or_assign("ObjectConflictId", std::to_string(current_conflicts_at_standstill.front().object_id));
    object_conflict_free_cycles_ = 0;
    RCLCPP_INFO(this->get_logger(), "Reduced reference speed cap to 0.0 m/s; object %lu still conflicts at standstill",
                current_conflicts_at_standstill.front().object_id);
  }
  route_plan.path.points.clear();
}

void SimplePlannerNode::appendRoutePoints(const route_planning_msgs::msg::Route& tf_route, FollowRoutePlan& route_plan,
                                          std::map<uint64_t, uint64_t>& lane_change_indices_map) {
  double t_total = 0.0;
  RCLCPP_DEBUG(this->get_logger(), "Number of remaining route elements: %zu", tf_route.destination_route_element_idx - tf_route.current_route_element_idx);
  health_.key_value_pairs.insert({"RemainingRouteElements", std::to_string(tf_route.destination_route_element_idx - tf_route.current_route_element_idx)});
  for (size_t j = tf_route.current_route_element_idx; j < tf_route.destination_route_element_idx; ++j) {
    const auto& route_element = tf_route.route_elements[j];
    if (!route_element.is_enriched) {
      RCLCPP_DEBUG(this->get_logger(), "Route element %zu is not enriched. Skipping.", j);
      continue;
    }

    const auto& suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(route_element);
    SimplePathPoint simple_path_point;
    simple_path_point.position = Eigen::Vector2d(suggested_lane.reference_pose.position.x, suggested_lane.reference_pose.position.y);
    simple_path_point.s = route_element.s;
    simple_path_point.v = v_ref_;
    if (v_ref_ < 0.0) {
      simple_path_point.v = suggested_lane.speed_limit / 3.6;
    }

    if (!route_plan.path.points.empty()) {
      double v_average = (route_plan.path.points.back().v + simple_path_point.v) / 2.0;
      double dt = 0.0;
      if (v_average != 0.0) {
        dt = (simple_path_point.s - route_plan.path.points.back().s) / v_average;
      }
      if (dt <= 0.0) {
        std::string msg = "Negative time difference " + std::to_string(dt) + " between points at s=" + std::to_string(route_plan.path.points.back().s) + " and s=" + std::to_string(simple_path_point.s) + ". Could lead to unexpected behavior.";
        setHealth(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg, health_.key_value_pairs);
        RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
      }
      t_total += dt;
    }

    if (j == tf_route.current_route_element_idx) {
      route_plan.suggested_turn_signal = suggested_lane.suggested_turn_signal;
    }

    if (route_element.will_change_suggested_lane && !tryRegisterLaneChange(tf_route, j, lane_change_indices_map, route_plan.suggested_turn_signal)) {
      break;
    }

    if (consider_traffic_lights_) {
      updateForTrafficLights(tf_route, j, suggested_lane, simple_path_point, t_total,
                             route_plan.stop_at_end, route_plan.offset_to_stop_line);
      if (route_plan.stop_at_end) route_plan.reason_to_stop = "Traffic light indicates stop";
    }

    route_plan.path.points.push_back(simple_path_point);

    if (j == tf_route.destination_route_element_idx - 1) {
      SimplePathPoint destination_point;
      destination_point.position = Eigen::Vector2d(tf_route.destination.x, tf_route.destination.y);
      destination_point.s = simple_path_point.s + (destination_point.position - simple_path_point.position).norm();
      destination_point.v = simple_path_point.v;
      route_plan.path.points.push_back(destination_point);
      route_plan.reason_to_stop = "Reaching end of route";
      route_plan.stop_at_end = true;
    }
    if (route_plan.stop_at_end || t_total >= 2.0 * trajectory_horizon_) {
      break;
    }
  }
}

bool SimplePlannerNode::tryRegisterLaneChange(const route_planning_msgs::msg::Route& tf_route, size_t route_element_idx,
                                              std::map<uint64_t, uint64_t>& lane_change_indices_map, uint8_t& suggested_turn_signal) {
  if (route_element_idx + 1 >= tf_route.route_elements.size()) {
    std::string msg = "Route element " + std::to_string(route_element_idx) + " is the last element. Cannot change lane.";
    setHealth(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg, health_.key_value_pairs);
    RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
    return false;
  }

  int i_end = route_element_idx + 1;
  const auto& route_element = tf_route.route_elements[route_element_idx];
  size_t current_lane_idx = route_element.suggested_lane_idx;
  int lane_change_direction = 0;
  try {
    lane_change_direction = route_planning_msgs::route_access::getLaneChangeDirection(route_element, tf_route.route_elements[route_element_idx + 1]);
  } catch (const std::exception& ex) {
    RCLCPP_WARN(this->get_logger(), "Could not determine lane change direction at route element %zu: %s", route_element_idx, ex.what());
    return false;
  }
  if (lane_change_direction < 0) {
    suggested_turn_signal = route_planning_msgs::msg::LaneElement::SUGGESTED_TURN_SIGNAL_LEFT;
  } else if (lane_change_direction > 0) {
    suggested_turn_signal = route_planning_msgs::msg::LaneElement::SUGGESTED_TURN_SIGNAL_RIGHT;
  }

  double ego_velocity = perception_msgs::object_access::getVelocityMagnitude(ego_data_);
  double lane_change_distance = std::max(lane_change_min_distance_factor_ * ego_data_.length, lane_change_distance_factor_ * ego_velocity);
  RCLCPP_INFO(this->get_logger(), "Lane change direction: %d, lane change distance: %f", lane_change_direction, lane_change_distance);
  if (lane_change_direction == 0) {
    RCLCPP_WARN(this->get_logger(), "Route element %zu is marked as lane change, but suggested lane does not change. Ignoring lane change marker.",
                route_element_idx);
    return true;
  }

  double ds = 0.0;
  int i_start = route_element_idx;
  while (ds < lane_change_distance && i_start > 0) {
    if (auto result = route_planning_msgs::route_access::getPrecedingLaneElementIdx(current_lane_idx, tf_route.route_elements[i_start - 1])) {
      current_lane_idx = *result;
    } else {
      std::string msg = "No preceding lane element found for route element " + std::to_string(i_start) + ". Cannot extend lane change.";
      RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
      setHealth(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg, health_.key_value_pairs);
      break;
    }
    if ((i_start - 2) >= 0 && tf_route.route_elements[i_start - 2].will_change_suggested_lane) {
      std::string msg = "Found previous lane change in route element " + std::to_string(i_start - 2) + ". Could not extend lane change over this element.";
      RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
      setHealth(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg, health_.key_value_pairs);
      break;
    }
    if (!route_planning_msgs::route_access::hasAdjacentLane(tf_route.route_elements[i_start - 1], current_lane_idx, lane_change_direction)) {
      std::string msg = "No adjacent lane found for route element " + std::to_string(i_start - 1) + ".";
      RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
      setHealth(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg, health_.key_value_pairs);
      break;
    }
    ds += std::abs(tf_route.route_elements[i_start].s - tf_route.route_elements[i_start - 1].s);
    i_start--;
  }

  if (!tf_route.route_elements[i_start].is_enriched || !tf_route.route_elements[i_end].is_enriched) {
    std::string msg = "Not enough enriched route elements (" + std::to_string(i_start) + ", " + std::to_string(i_end) + ") for lane change.";
    RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
    setHealth(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg, health_.key_value_pairs);
    return false;
  }

  lane_change_indices_map[route_element_idx] = i_start;
  return true;
}

void SimplePlannerNode::updateForTrafficLights(const route_planning_msgs::msg::Route& tf_route, size_t route_element_idx,
                                               const route_planning_msgs::msg::LaneElement& suggested_lane,
                                               const SimplePathPoint& simple_path_point, double t_total,
                                               bool& stop_at_end, double& offset_to_stop_line) {
  const auto& route_element = tf_route.route_elements[route_element_idx];
  const auto& reg_elems = route_planning_msgs::route_access::getRegulatoryElementsOfLaneElement(suggested_lane, route_element.regulatory_elements);
  for (size_t k = 0; k < reg_elems.size(); ++k) {
    if (reg_elems[k].type != route_planning_msgs::msg::RegulatoryElement::TYPE_TRAFFIC_LIGHT) {
      continue;
    }
    if (reg_elems[k].meta_value == route_planning_msgs::msg::RegulatoryElement::META_VALUE_MOVEMENT_ALLOWED && !consider_future_states_) {
      continue;
    }

    offset_to_stop_line = offset_to_stop_line_ + ego_data_.length / 2.0 + ego_data_.state.reference_point.translation_to_geometric_center.x;
    double dt_offset_to_stop_line = 0.0;
    if (simple_path_point.v != 0.0) {
      dt_offset_to_stop_line = offset_to_stop_line / simple_path_point.v;
    }
    if (dt_offset_to_stop_line <= 0.0) {
      std::string msg = "Negative time difference 'dt_offset_to_stop_line' (" + std::to_string(dt_offset_to_stop_line) + " s) for traffic light at route element " + std::to_string(route_element_idx) + ". Could lead to unexpected behavior.";
      setHealth(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg, health_.key_value_pairs);
      RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
    }

    if (reg_elems[k].has_validity_stamp && consider_future_states_) {
      double validity_duration = rclcpp::Time(reg_elems[k].validity_stamp).seconds() - rclcpp::Time(route_.header.stamp).seconds();
      if (validity_duration < (t_total - dt_offset_to_stop_line)) {
        if (reg_elems[k].meta_value == route_planning_msgs::msg::RegulatoryElement::META_VALUE_MOVEMENT_ALLOWED) {
          stop_at_end = true;
        } else {
          continue;
        }
      } else {
        if (reg_elems[k].meta_value == route_planning_msgs::msg::RegulatoryElement::META_VALUE_MOVEMENT_ALLOWED) {
          continue;
        } else {
          stop_at_end = true;
        }
      }
    } else {
      stop_at_end = true;
    }

    double distance_to_stop_point = tf_route.route_elements[route_element_idx].s - tf_route.route_elements[tf_route.current_route_element_idx].s - offset_to_stop_line;
    double v_ego = perception_msgs::object_access::getVelocityMagnitude(ego_data_);
    double min_distance_to_stop = -0.5 * std::pow(v_ego, 2) / a_max_decel_;
    if (distance_to_stop_point < 0.0 && std::abs(distance_to_stop_point) > ignore_stop_line_threshold_) {
      std::string msg = "Traffic light stop point is behind ego vehicle (distance to stop point: " + std::to_string(distance_to_stop_point) + " m). Ignoring traffic light.";
      setHealth(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg, health_.key_value_pairs);
      RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
      stop_at_end = false;
    } else if ((distance_to_stop_point < min_distance_to_stop) && stop_at_end) {
      std::string msg = "Traffic light requires stop, but distance to stop point is smaller than minimum distance to stop. Ignoring traffic light.";
      setHealth(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg, health_.key_value_pairs);
      RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
      stop_at_end = false;
    }
  }
}

std::vector<SimplePathPoint> SimplePlannerNode::mergeLaneChangeSegments(const route_planning_msgs::msg::Route& tf_route,
                                                                        const std::vector<SimplePathPoint>& route_points,
                                                                        const std::map<uint64_t, uint64_t>& lane_change_indices_map) {
  size_t current = 0;
  std::vector<SimplePathPoint> merged_points;
  for (const auto& lane_change_indices : lane_change_indices_map) {
    uint64_t lane_change_idx_route = lane_change_indices.first;
    uint64_t start_idx_route = lane_change_indices.second;
    size_t start_idx = start_idx_route > tf_route.current_route_element_idx ? start_idx_route - tf_route.current_route_element_idx : 0;
    size_t end_idx = lane_change_idx_route + 1 > tf_route.current_route_element_idx ? lane_change_idx_route + 1 - tf_route.current_route_element_idx : 0;

    if (start_idx < current || current > route_points.size()) {
      RCLCPP_WARN(this->get_logger(), "Skipping overlapping lane change window (%zu, %zu), current path index: %zu.",
                  start_idx, end_idx, current);
      continue;
    }

    start_idx = std::min(start_idx, route_points.size());
    merged_points.insert(merged_points.end(), route_points.begin() + current, route_points.begin() + start_idx);
    std::vector<SimplePathPoint> lane_change_points = generateLaneChangePath(start_idx_route, lane_change_idx_route, tf_route);
    merged_points.insert(merged_points.end(), lane_change_points.begin(), lane_change_points.end());
    current = std::min(end_idx + 1, route_points.size());
  }
  if (current <= route_points.size()) {
    merged_points.insert(merged_points.end(), route_points.begin() + current, route_points.end());
  }
  return merged_points;
}

void SimplePlannerNode::applyIndicatorRequest(uint8_t suggested_turn_signal) {
  if (suggested_turn_signal == route_planning_msgs::msg::LaneElement::SUGGESTED_TURN_SIGNAL_NONE &&
      left_turn_indicator_service_client_->service_is_ready() && right_turn_indicator_service_client_->service_is_ready() &&
      hazard_lights_service_client_->service_is_ready()) {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;
    left_turn_indicator_service_client_->async_send_request(request);
    right_turn_indicator_service_client_->async_send_request(request);
    hazard_lights_service_client_->async_send_request(request);
  } else if (suggested_turn_signal == route_planning_msgs::msg::LaneElement::SUGGESTED_TURN_SIGNAL_LEFT && left_turn_indicator_service_client_->service_is_ready()) {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    left_turn_indicator_service_client_->async_send_request(request);
  } else if (suggested_turn_signal == route_planning_msgs::msg::LaneElement::SUGGESTED_TURN_SIGNAL_RIGHT && right_turn_indicator_service_client_->service_is_ready()) {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    right_turn_indicator_service_client_->async_send_request(request);
  } else if (suggested_turn_signal == route_planning_msgs::msg::LaneElement::SUGGESTED_TURN_SIGNAL_HAZARD && hazard_lights_service_client_->service_is_ready()) {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    hazard_lights_service_client_->async_send_request(request);
  } else {
    std::string msg = "Cannot apply suggested turn signal " + std::to_string(suggested_turn_signal) + " because the corresponding indicator service is not ready.";
    RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
    setHealth(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg, health_.key_value_pairs);
  }
}

void SimplePlannerNode::trimPathBehindEgo(SimplePath& path) {
  while (!path.points.empty() && path.points[0].position.x() < 0.0) {
    path.points.erase(path.points.begin());
  }
}

SimplePath SimplePlannerNode::calculateSafeStopAlongRoute(const SimplePath& path, const double safe_stop_distance) {
  SimplePath safe_stop_path = path;
  double start_s = safe_stop_path.points[0].s; // start s value of path
  for (size_t i = 0; i < safe_stop_path.points.size(); i++) {
    if (safe_stop_path.points[i].s > start_s + safe_stop_distance) {
      // remove all points after the point where the safe stop distance is reached
      safe_stop_path.points.erase(safe_stop_path.points.begin() + i, safe_stop_path.points.end());
      break;
    }
  }
  safe_stop_path.points = resamplePath(safe_stop_path.points, true, 0.0);
  return safe_stop_path;
}

SimplePath SimplePlannerNode::calculateSafeStopAlongEgoHeading(const perception_msgs::msg::EgoData& ego_data, const double safe_stop_distance, const std_msgs::msg::Header& target_header) {
  SimplePath safe_stop_path;
  safe_stop_path.header = ego_data.header;

  double v_ego = perception_msgs::object_access::getVelocityMagnitude(ego_data);
  geometry_msgs::msg::Point point = perception_msgs::object_access::getPosition(ego_data);
  double yaw = perception_msgs::object_access::getYaw(ego_data);
  Eigen::Vector2d start_position(point.x, point.y);
  Eigen::Vector2d heading(std::cos(yaw), std::sin(yaw));

  RCLCPP_WARN(this->get_logger(), "Initializing minimal safe stop along ego heading. Frame: %s, yaw: %f rad",
              safe_stop_path.header.frame_id.c_str(), yaw);
  (void)target_header;

  SimplePathPoint start_point(start_position, 0.0, v_ego);
  safe_stop_path.points.push_back(start_point);
  SimplePathPoint mid_point(start_position + heading * (safe_stop_distance / 2.0), safe_stop_distance / 2.0, v_ego);
  safe_stop_path.points.push_back(mid_point);
  SimplePathPoint end_point(start_position + heading * safe_stop_distance, safe_stop_distance, 0.0);
  safe_stop_path.points.push_back(end_point);

  safe_stop_path.points = resamplePath(safe_stop_path.points, true, 0.0);
  return safe_stop_path;
}

std::vector<SimplePathPoint> SimplePlannerNode::generateLaneChangePath(const int start_idx, const int turn_idx,
                                                                       const route_planning_msgs::msg::Route& route) {
  std::vector<SimplePathPoint> lane_change_path;
  int end_idx = turn_idx + 1; // lane change should end at the next route element
  if (start_idx >= end_idx || start_idx < 0 || end_idx >= static_cast<int>(route.route_elements.size())) {
    std::string msg = "Invalid lane change indices: start_idx=" + std::to_string(start_idx) + ", end_idx=" + std::to_string(end_idx) + ". Cannot generate lane change path.";
    RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
    setHealth(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg, health_.key_value_pairs);
    return lane_change_path;
  }

  int lane_change_direction = route_planning_msgs::route_access::getLaneChangeDirection(route.route_elements[turn_idx], route.route_elements[turn_idx + 1]);

  // Interpolate between the two elements
  for (int i = start_idx; i <= end_idx; ++i) {
    if (i < static_cast<int>(route.current_route_element_idx)) continue;

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

std::vector<SimplePathPoint> SimplePlannerNode::resamplePath(const std::vector<SimplePathPoint>& path, bool stop_at_end, double offset_to_stop_line,
                                                             const double* speed_cap) {
  rclcpp::Time begin = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  if (path.empty()) {
    std::string msg = "Route is empty. No resampling possible.";
    RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
    setHealth(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg, health_.key_value_pairs);
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
      if (s >= path[j].s && s <= path[j + 1].s) {
        idx = static_cast<int>(j);
        break;
      }
    }

    double v = v_ref_; // option 1: use predefined constant velocity
    if (v_ref_ < 0.0 && idx >= 0) { // option 2: use velocity from route if predefined velocity is negative
      v = path[idx].v + (path[idx + 1].v - path[idx].v) / (path[idx + 1].s - path[idx].s) * (s - path[idx].s);
    }
    if (speed_cap != nullptr) {
      v = std::min(v, std::max(*speed_cap, 0.0));
    }

    double distance_to_stop = -0.5 * std::pow(v, 2) / a_decel_ + offset_to_stop_line;
    distance_to_stop = std::max(distance_to_stop, 0.0);
    double brake_point = path.back().s - distance_to_stop;
    double ds = v * dt_; // case 1: constant velocity
    if (s + ds > brake_point && stop_at_end) {
      if (s < brake_point) { // special case: braking point is between two states
        double ds_1 = brake_point - s; // distance with constant velocity to braking point
        double dt_1 = ds_1 / v; // time with constant velocity to braking point
        double dt_2 = dt_ - dt_1; // remaining time with deceleration
        double ds_2 = std::max(0.5 * a_decel_ * std::pow(dt_2, 2) + v * dt_2, 0.0);
        ds = ds_1 + ds_2;
      } else {
        v = std::sqrt(std::max(std::pow(v, 2) + 2 * a_decel_ * (s - brake_point), 0.0)); // case 2: deceleration (v(s))
        ds = 0.5 * a_decel_ * std::pow(dt_, 2) + v * dt_; // case 2: deceleration
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
      simple_path_point.position.x() = path[idx].position.x() + (path[idx + 1].position.x() - path[idx].position.x()) / (path[idx + 1].s - path[idx].s) * (s - path[idx].s);
      simple_path_point.position.y() = path[idx].position.y() + (path[idx + 1].position.y() - path[idx].position.y()) / (path[idx + 1].s - path[idx].s) * (s - path[idx].s);
    }
    else { // unsupported interpolation type
      RCLCPP_ERROR(this->get_logger(), "Unsupported interpolation type value %d", interpolation_type_);
      throw std::runtime_error("Unsupported interpolation type value");
    }
    simple_path_point.s = s;
    simple_path_point.v = v;
    resampled_path.push_back(simple_path_point);

    // increment s and v for next iteration
    if (v <= 1e-6 && ds <= 1e-6) break;
    s = s + ds;
    if (s == path.back().s && v == 0.0) break; // stop at end of route
  }

  rclcpp::Time end = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  RCLCPP_DEBUG(this->get_logger(), "Resampling route took %f ms", (end - begin).seconds() * 1e3);

  if (stop_at_end) {
    SimplePathPoint stop_point = path.back();
    if (!resampled_path.empty() && (offset_to_stop_line > 0.0 || (speed_cap != nullptr && *speed_cap <= 1e-6))) {
      stop_point = resampled_path.back();
    }
    stop_point.v = 0.0;
    if (resampled_path.empty() || resampled_path.back().s < stop_point.s || resampled_path.back().v != 0.0) {
      resampled_path.push_back(stop_point);
    }
  }

  return resampled_path;
}

/**
 * @brief This callback is invoked every period seconds by the timer
 *
 */
void SimplePlannerNode::publishTimerCallback() {
  const rclcpp::Time stamp = now();
  PlannerState planner_state = determinePlannerState(stamp);
  if (planner_state == PlannerState::NoPublish) {
    std_msgs::msg::Header marker_header;
    marker_header.stamp = stamp;
    marker_header.frame_id = trajectory_frame_id_;
    clearObjectInteractionMarkers(marker_header);
    diagnostic_updater_.force_update();
    return;
  }

  try {
    trajectory_planning_msgs::msg::Trajectory msg = createTrajectory(planner_state, stamp);
    diagnosed_publisher_->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "Published Trajectory!");
  } catch (const std::runtime_error& e) {
    std::string msg = "Error while creating trajectory: " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
    setHealth(diagnostic_msgs::msg::DiagnosticStatus::ERROR, msg, { {"PlannerState", plannerStatetoString(planner_state)} });
  }
  diagnostic_updater_.force_update();
}

}  // namespace simple_planner

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_planner::SimplePlannerNode>());
  rclcpp::shutdown();

  return 0;
}
