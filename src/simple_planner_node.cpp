#include <chrono>
#include <cmath>
#include <functional>
#include <thread>

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

  this->declareAndLoadParameters("trajectory_frame_id", trajectory_frame_id_, rclcpp::ParameterType::PARAMETER_STRING,
                                 "Frame ID of published reference trajectory", true);
  this->declareAndLoadParameters("fixed_over_time_frame_id", fixed_over_time_frame_id_, rclcpp::ParameterType::PARAMETER_STRING,
                                  "Frame ID of frame that is fixed over time for finding temporal transforms", true);
  this->declareAndLoadParameters("frequency", freq_, rclcpp::ParameterType::PARAMETER_DOUBLE, "frequency of publishing trajectory", true);
  this->declareAndLoadParameters("drivable_mode", drivable_mode_, rclcpp::ParameterType::PARAMETER_BOOL, "true: creating drivable trajectory; false: creating reference trajectory", true);
  this->declareAndLoadParameters("n_states", n_states_, rclcpp::ParameterType::PARAMETER_INTEGER, "number of states in the trajectory", true);
  this->declareAndLoadParameters("v_ref", v_ref_, rclcpp::ParameterType::PARAMETER_DOUBLE, "reference velocity (m/s); set for all states in the trajectory", true);
  this->declareAndLoadParameters("a_max_decel", a_max_decel_, rclcpp::ParameterType::PARAMETER_DOUBLE, "maximum deceleration (m/s^2) - must be < 0.0", true);
  this->declareAndLoadParameters("consider_traffic_lights", consider_traffic_lights_, rclcpp::ParameterType::PARAMETER_BOOL, "true: planner will consider traffic lights; false: planner will ignore traffic lights", true);
  this->declareAndLoadParameters("offset_to_stop_line", offset_to_stop_line_, rclcpp::ParameterType::PARAMETER_DOUBLE, "additional distance to stop in front of a stop line (m) (default: 0.0 -> stops with front of vehicle at stop line)", true);

  this->setup();
}

template <typename T>
void SimplePlannerNode::declareAndLoadParameters(const std::string& name, T& member_param,
                                                 const rclcpp::ParameterType& type, const std::string& description,
                                                 const bool& add_to_reconfigurable_node_params) {
  
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = description;
  
  this->declare_parameter(name, type, param_desc);

  try {
    if constexpr (std::is_same_v<T, std::string>) {
      member_param = this->get_parameter(name).as_string();
    } else if constexpr (std::is_same_v<T, double>) {
      member_param = this->get_parameter(name).as_double();
    } else if constexpr (std::is_same_v<T, bool>) {
      member_param = this->get_parameter(name).as_bool();
    } else if constexpr (std::is_same_v<T, int>) {
      member_param = this->get_parameter(name).as_int();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Parameter type not supported.");
    }
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter '" << name << "' not set. Using default value: " << member_param);
  } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Invalid parameter type for '" << name << "'. Using default value: " << member_param);
  } catch (rclcpp::exceptions::InvalidParameterValueException&) {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Invalid parameter value for '" << name << "'. Using default value: " << member_param);
  }  

  if (add_to_reconfigurable_node_params) {
    nodeParams_.push_back(std::make_tuple(name, &member_param, type, description));
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
    for (auto& nodeParam : nodeParams_) {
      if (param.get_name() == std::get<0>(nodeParam)) {
        void* memberParamPtr = std::get<1>(nodeParam);
        rclcpp::ParameterType paramType = std::get<2>(nodeParam);

        if (paramType == rclcpp::ParameterType::PARAMETER_STRING) {
          *static_cast<std::string*>(memberParamPtr) = param.as_string();
        } else if (paramType == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          *static_cast<double*>(memberParamPtr) = param.as_double();
        } else if (paramType == rclcpp::ParameterType::PARAMETER_BOOL) {
          *static_cast<bool*>(memberParamPtr) = param.as_bool();
        } else if (paramType == rclcpp::ParameterType::PARAMETER_INTEGER) {
          *static_cast<int*>(memberParamPtr) = param.as_int();
        } else {
          RCLCPP_ERROR(this->get_logger(), "Parameter type not supported.");
        }
      }
    }
  }

  // mark parameter change successful
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

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

  if (!route_init_) {
    route_init_ = true;
    s_start_brake_ = route_.remaining_route.back().z - distance_to_stop_;
    RCLCPP_INFO(this->get_logger(), "Received first route message, start beak s: %f", s_start_brake_);
  }
}

trajectory_planning_msgs::msg::Trajectory SimplePlannerNode::createTrajectory() {
  // define trajectory message and set header
  int type_id =
      drivable_mode_ ? trajectory_planning_msgs::DRIVABLE::TYPE_ID : trajectory_planning_msgs::REFERENCE::TYPE_ID;
  trajectory_planning_msgs::msg::Trajectory tra;
  tra.header.stamp = now();
  tra.header.frame_id = trajectory_frame_id_;

  // TODO: additionally check if destination is reached or if route is outdated?
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
  next_braking_point = next_braking_point - (ego_data_.length / 2.0 + ego_data_.state.reference_point.translation_to_geometric_center.x);

  // saving remaining route in path and checking if path starts behind trajectory_frame_id_, which could cause unintended behavior for drivable trajectories
  std::vector<geometry_msgs::msg::Point> path = tf_route.remaining_route;
  if (path[0].x < 0.0)
    RCLCPP_DEBUG(this->get_logger(), "Path starts %f m behind %s. Could cause unintended behavior.", path[0].x, trajectory_frame_id_.c_str());
  if (drivable_mode_) path.insert(path.begin(), geometry_msgs::msg::Point());

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
    double v = v_ref_;
    if (path[i].z >= next_braking_point) {
      v = std::sqrt(std::max(std::pow(v_ref_, 2) + 2 * a_max_decel_ * (path[i].z - next_braking_point), 0.0));  // decelerate to stop at ref line
    }
    trajectory_planning_msgs::trajectory_access::setT(tra, calcDistance(path, i) / v_ref_, i);
    trajectory_planning_msgs::trajectory_access::setX(tra, path[i].x, i);
    trajectory_planning_msgs::trajectory_access::setY(tra, path[i].y, i);
    trajectory_planning_msgs::trajectory_access::setV(tra, v, i);
    if (drivable_mode_) {
      trajectory_planning_msgs::trajectory_access::setS(tra, calcDistance(path, i), i);
      trajectory_planning_msgs::trajectory_access::setTheta(tra, calcTheta(path, i), i);
      // TODO: setA, setKappa, setDkappa
    }
    RCLCPP_DEBUG(this->get_logger(), "Debug: i: %ld,  t: %f,  x: %f,  y: %f,  v: %f, s: %f,  theta: %f", i,
                 calcDistance(path, i) / v_ref_, path[i].x, path[i].y, v, calcDistance(path, i), calcTheta(path, i));
  }
  trajectory_planning_msgs::trajectory_access::setStandstill(tra, false);

  RCLCPP_DEBUG(this->get_logger(), "Standstill = %d", tra.standstill);
  return tra;
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