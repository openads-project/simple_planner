#include <chrono>
#include <cmath>
#include <functional>
#include <thread>
#include <vector>

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
  this->declareAndLoadParameter("static_route", static_route_,
                                "true: incoming route/path is static; false: incoming route/path is dynamic");
  this->declareAndLoadParameter("trajectory_horizon", trajectory_horizon_, "time horizon of the reference trajectory (s)");
  this->declareAndLoadParameter("n_states", n_states_, "number of states in the trajectory");
  this->declareAndLoadParameter("v_ref", v_ref_, "reference velocity (m/s); set for all states in the trajectory");
  this->declareAndLoadParameter("a_max_decel", a_max_decel_, "maximum deceleration (m/s^2) - must be < 0.0");
  this->declareAndLoadParameter(
      "consider_traffic_lights", consider_traffic_lights_,
      "true: planner will consider traffic lights; false: planner will ignore traffic lights");
  this->declareAndLoadParameter("offset_to_stop_line", offset_to_stop_line_,
                                "additional distance to stop in front of a stop line (m) (default: 0.0 -> stops with "
                                "front of vehicle at stop line)");

  this->setup();
}

template <typename T>
void SimplePlannerNode::declareAndLoadParameter(const std::string& name, T& member_param,
                                                const std::string& description,
                                                const bool add_to_auto_reconfigurable_params, const bool is_required,
                                                const bool read_only, const std::optional<T>& from_value,
                                                const std::optional<T>& to_value, const std::optional<T>& step_value,
                                                const std::string& additional_constraints) {
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = description;
  param_desc.additional_constraints = additional_constraints;
  param_desc.read_only = read_only;

  auto param_type = rclcpp::ParameterValue(member_param).get_type();

  if (from_value.has_value() && to_value.has_value()) {
    if constexpr (std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      T step = step_value.has_value() ? step_value.value() : 0;
      range.set__from_value(from_value.value()).set__to_value(to_value.value()).set__step(step);
      param_desc.integer_range = {range};
    } else if constexpr (std::is_floating_point_v<T>) {
      rcl_interfaces::msg::FloatingPointRange range;
      T step = step_value.has_value() ? step_value.value() : 0.0;
      range.set__from_value(from_value.value()).set__to_value(to_value.value()).set__step(step);
      param_desc.floating_point_range = {range};
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter type does not support range.");
    }
  }

  this->declare_parameter(name, param_type, param_desc);

  try {
    member_param = this->get_parameter(name).get_value<T>();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    if (is_required) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Parameter '" << name << "' not set but required. Exiting.");
      exit(EXIT_FAILURE);
    } else {
      std::stringstream ss;
      ss << "Parameter '" << name << "' not set. Using default value: ";
      if constexpr (is_vector_v<T>) {
        ss << "[";
        for (const auto& element : member_param) ss << element << (&element != &member_param.back() ? ", " : "]");
      } else {
        ss << member_param;
      }
      RCLCPP_WARN_STREAM(this->get_logger(), ss.str());
    }
  }

  if (add_to_auto_reconfigurable_params) {
    std::function<void(const rclcpp::Parameter&)> setter = [&member_param](const rclcpp::Parameter& param) {
      member_param = param.get_value<T>();
    };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}

/**
 * @brief Handles reconfiguration when a parameter value is changed
 *
 * @param parameters parameters
 * @return parameter change result
 */
rcl_interfaces::msg::SetParametersResult SimplePlannerNode::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
      }
    }
  }

  // mark parameter change successful
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

  // define distance to stop
  if (a_max_decel_ < 0.0) {
    distance_to_stop_ = -0.5 * std::pow(v_ref_, 2) / a_max_decel_;
  } else {
    distance_to_stop_ = 0.0;
  }

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
  s_start_brake_ = route_.remaining_route.back().z - distance_to_stop_;
  RCLCPP_INFO(this->get_logger(), "Received route message, initialized global variable");
  RCLCPP_ERROR(this->get_logger(), "s_start_brake_: %f, end of route: %f", s_start_brake_, route_.remaining_route.back().z);
  resampleRoute(route_);
  if (!route_init_) route_init_ = true;
}

trajectory_planning_msgs::msg::Trajectory SimplePlannerNode::createTrajectory() {
  // define trajectory message and set header
  int type_id = trajectory_planning_msgs::REFERENCE::TYPE_ID;
  trajectory_planning_msgs::msg::Trajectory tra;
  tra.header.stamp = now();
  tra.header.frame_id = trajectory_frame_id_;

  // TODO: additionally check if destination is reached or route is outdated?
  if (route_.remaining_route.empty()) {
    trajectory_planning_msgs::trajectory_access::initializeTrajectory(tra, type_id, 1);
    route_init_ = false;
    RCLCPP_WARN(this->get_logger(), "Remaining route empty -> destination reached. Publishing standstill trajectory.");
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
  // Currently doesn't work
  // route_planning_msgs::msg::Route tf_route;
  // try {
  //   tf_route = tf2_buffer_->transform(route_, tra.header.frame_id, tf2_ros::fromMsg(tra.header.stamp),
  //                                     fixed_over_time_frame_id_, tf2::durationFromSec(0.01));
  // } catch (tf2::TransformException& ex) {
  //   RCLCPP_WARN(this->get_logger(), "Could not transform route: %s", ex.what());
  // }

  // find next traffic light stop line and calculate braking point
  double next_stop_line = std::numeric_limits<double>::infinity();
  if (consider_traffic_lights_){
    for(size_t j = 0; j < tf_route.regulatory_elements.size(); ++j) {
      if (tf_route.regulatory_elements[j].type != route_planning_msgs::msg::RegulatoryElement::TYPE_TRAFFIC_LIGHT) continue;
      if (tf_route.regulatory_elements[j].value == route_planning_msgs::msg::RegulatoryElement::MOVEMENT_ALLOWED) continue;
      if (tf_route.regulatory_elements[j].effect_line[0].z >= 0.0 && tf_route.regulatory_elements[j].effect_line[0].z < next_stop_line) {
        next_stop_line = tf_route.regulatory_elements[j].effect_line[0].z;
      }
    }
    next_stop_line = next_stop_line - offset_to_stop_line_;
    RCLCPP_DEBUG(this->get_logger(), "Next stop line at s: %f (global)", next_stop_line);
  }

  double next_braking_point = std::min(s_start_brake_, next_stop_line - distance_to_stop_);
  // make sure to stop with the front of the vehicle at the stop line
  next_braking_point = next_braking_point - (ego_data_.length / 2.0 + ego_data_.state.reference_point.translation_to_geometric_center.x);// TODO: not used? 

  // saving remaining route in path and checking if path starts behind trajectory_frame_id_, which could cause unintended behavior for drivable trajectories
  std::vector<geometry_msgs::msg::Point> path = tf_route.remaining_route;
  // search for closest point index in route to ego vehicle
  size_t closest_index = 0;
  double min_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 1; i < path.size(); i++) {

    
    if (path[i].z - s_ > max_distance_of_interest_) break; // ignore points in front of ego vehicle, TODO: magic number

    double distance = std::sqrt(std::pow(path[i].x, 2) + std::pow(path[i].y, 2));
    if (distance < min_distance) {
      min_distance = distance;
      closest_index = i;
      if (distance < min_distance_of_interest_) break; // TODO: magic number
    }
  }

  // remove all points but one before closest point with x < 0
  for (size_t i = closest_index; i > 0; i--) {
    if (path[i].x < 0.0) {
      path.erase(path.begin(), path.begin() + i + 1);
      v_profile_.erase(v_profile_.begin(), v_profile_.begin() + i + 1);
      break;
    }
  }

  // update s_ with closest point
  s_ = path[closest_index].z;

  // save updated path in member variable
  if (static_route_) {
    route_.header = tra.header;
    route_.remaining_route = path;
  }

  // currently unused - might be useful for publishing drivable trajectories -> only point where ego_data_ is used
  // geometry_msgs::msg::Pose current_pose = perception_msgs::object_access::getPose(ego_data_);
  // double current_velocity = perception_msgs::object_access::getVelocityMagnitude(ego_data_);
  // double current_speed_limit = tf_route.current_speed_limit/3.6;

  // keep maximum the first n_states_ in path (and therefore in trajectory)
  if ((size_t) n_states_ < path.size()) {
    path.erase(path.begin() + n_states_, path.end());
  }

  // init trajectory and fill with path (route) and velocity (const from param) data
  trajectory_planning_msgs::trajectory_access::initializeTrajectory(tra, type_id, path.size());
  for (size_t i = 0; i < path.size(); i++) {
    trajectory_planning_msgs::trajectory_access::setT(tra, dt_ * i, i);
    trajectory_planning_msgs::trajectory_access::setX(tra, path[i].x, i);
    trajectory_planning_msgs::trajectory_access::setY(tra, path[i].y, i);
    trajectory_planning_msgs::trajectory_access::setV(tra, v_profile_[i], i);
    // TODO: maybe add last point multiple times to reach n_states_
    RCLCPP_DEBUG(this->get_logger(), "Debug: i: %ld,  t: %f,  x: %f,  y: %f,  v: %f, s: %f", i,
                 dt_ * i, path[i].x, path[i].y, v_profile_[i], path[i].z);
  }
  trajectory_planning_msgs::trajectory_access::setStandstill(tra, false);

  RCLCPP_DEBUG(this->get_logger(), "Standstill = %d", tra.standstill);
  return tra;
}

void SimplePlannerNode::resampleRoute(route_planning_msgs::msg::Route& route) {
  v_profile_.clear();
  s_ = 0.0;
  std::vector<geometry_msgs::msg::Point> path;
  double s = 0.0;
  std::vector<double> z_vector, x_vector, y_vector;
  if (use_spline_interpolation_) {
    for (size_t j = 0; j < route.remaining_route.size(); ++j) {
      z_vector.push_back(route.remaining_route[j].z);
      x_vector.push_back(route.remaining_route[j].x);
      y_vector.push_back(route.remaining_route[j].y);
    }
  }
  tk::spline x_spline(z_vector, x_vector);
  tk::spline y_spline(z_vector, y_vector);
  double v = v_ref_; // case 1: constant velocity
  while (s<=route.remaining_route.back().z) {
    double ds = v_ref_ * dt_; // case 1: constant velocityv_ref: 3.0  
    int idx = -1;
    if (s + ds > s_start_brake_) { // TODO: what aboute next_braking_point?
      if (s < s_start_brake_) { // special case: braking point is between two states
        double ds_1 = s_start_brake_ - s; // distance with constant velocity to braking point
        double dt_1 = ds_1 / v_ref_; // time with constant velocity to braking point
        double dt_2 = dt_ - dt_1; // remaining time with deceleration
        double ds_2 = std::max(0.5 * a_max_decel_ * std::pow(dt_2, 2) + v_ref_ * dt_2, 0.0);
        ds = ds_1 + ds_2;
      } else {
        v = std::sqrt(std::max(std::pow(v_ref_, 2) + 2 * a_max_decel_ * (s - s_start_brake_), 0.0)); // case 2: deceleration (v(s))
        ds = 0.5 * a_max_decel_ * std::pow(dt_, 2) + v * dt_; // case 2: deceleration
        if (ds < 0.0) ds = route.remaining_route.back().z - s; // only add rest of route instead of driving backwards
      }
    }
    
    // find index of segment in route
    for (size_t j = 0; j < route.remaining_route.size() - 1; ++j) {
      if (s >= route.remaining_route[j].z && s <= route.remaining_route[j+1].z) {
        idx = j;
        break;
      }
    }
    
    // interpolate point at s
    geometry_msgs::msg::Point point;
    if (use_spline_interpolation_){
      point.x = x_spline(s);
      point.y = y_spline(s);
      point.z = s;
    }
    else {
      point.x = route.remaining_route[idx].x + (route.remaining_route[idx+1].x - route.remaining_route[idx].x) / (route.remaining_route[idx+1].z - route.remaining_route[idx].z) * (s - route.remaining_route[idx].z);
      point.y = route.remaining_route[idx].y + (route.remaining_route[idx+1].y - route.remaining_route[idx].y) / (route.remaining_route[idx+1].z - route.remaining_route[idx].z) * (s - route.remaining_route[idx].z);
      point.z = s;
    }
    path.push_back(point);
    v_profile_.push_back(v);

    // increment s and v for next iteration 
    s = s + ds;

    RCLCPP_WARN(this->get_logger(), "s: %f, v: %f, v_ref_: %f", s, v, v_ref_);
    if (s == route.remaining_route.back().z && v == 0.0) break; // stop at end of route
  }

  route.remaining_route = path;
}

bool SimplePlannerNode::isDestinationReached(const geometry_msgs::msg::Point& destination) {
  double distance = std::sqrt(std::pow(destination.x, 2) + std::pow(destination.y, 2));
  RCLCPP_DEBUG(this->get_logger(), "Distance to goal: %f", distance);
  return distance < 0.2;
}

double SimplePlannerNode::calcDistance(const std::vector<geometry_msgs::msg::Point>& points, const int& nPoint) {
  double distance = 0.0;
  for (int i = 0; i <= nPoint; i++) {
    if (i == 0) {
      distance += std::sqrt(std::pow(points[i].x - 0.0, 2) + std::pow(points[i].y - 0.0, 2));
    } else {
      distance += std::sqrt(std::pow(points[i].x - points[i - 1].x, 2) + std::pow(points[i].y - points[i - 1].y, 2));
    }
  }
  return distance;
}

double SimplePlannerNode::calcTheta(const std::vector<geometry_msgs::msg::Point>& points, const int& nPoint) {
  double theta = 0.0;
  for (int i = 0; i <= nPoint; i++) {
    if (i == 0) {
      // theta += atan2(points[i].y - 0.0, points[i].x - 0.0);
      theta += 0.0;
    } else {
      theta += atan2(points[i].y - points[i - 1].y, points[i].x - points[i - 1].x);
    }
  }
  return theta;
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

  trajectory_planning_msgs::msg::Trajectory msg = createTrajectory();

  pub_->publish(msg);
  RCLCPP_DEBUG(this->get_logger(), "Published Trajectory!");
}

}  // namespace simple_planner

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_planner::SimplePlannerNode>());
  rclcpp::shutdown();

  return 0;
}