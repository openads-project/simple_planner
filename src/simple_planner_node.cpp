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

// constants
const std::string SimplePlannerNode::kEgoDataTopic = "~/ego_data";
const std::string SimplePlannerNode::kRouteTopic = "~/route";
const std::string SimplePlannerNode::kOutputTopic = "~/trajectory";
const std::string SimplePlannerNode::kTrajectoryFrameParam = "trajectory_frame_id";
const std::string SimplePlannerNode::kFreqParam = "frequency";
const std::string SimplePlannerNode::kDriveModeParam = "drivable_mode";
const std::string SimplePlannerNode::kNStatesParam = "n_states";
const std::string SimplePlannerNode::kVRefParam = "v_ref";
const std::string SimplePlannerNode::kAMaxDecelParam = "a_max_decel";

/**
 * @brief Creates a SimplePlannerNode node
 *
 */
SimplePlannerNode::SimplePlannerNode() : Node("simple_planner_node") {
  this->loadParameters();
  this->setup();
}

/**
 * @brief Loads ROS parameters used in the node.
 *
 */
void SimplePlannerNode::loadParameters() {
  // set parameter description
  rcl_interfaces::msg::ParameterDescriptor trajectory_frame_param_desc;
  freq_param_desc.description = "frame_id of published reference trajectory";
  rcl_interfaces::msg::ParameterDescriptor freq_param_desc;
  freq_param_desc.description = "frequency of publishing trajectory";
  rcl_interfaces::msg::ParameterDescriptor driveMode_param_desc;
  driveMode_param_desc.description = "true: creating drivable trajectory; false: creating reference trajectory";
  rcl_interfaces::msg::ParameterDescriptor nStates_param_desc;
  nStates_param_desc.description = "number of states in the trajectory";
  rcl_interfaces::msg::ParameterDescriptor vRef_param_desc;
  vRef_param_desc.description = "reference velocity (m/s); set for all states in the trajectory";
  rcl_interfaces::msg::ParameterDescriptor aMaxDecel_param_desc;
  aMaxDecel_param_desc.description = "maximum deceleration (m/s^2) - must be < 0.0";

  // declare parameter
  this->declare_parameter(kTrajectoryFrameParam, rclcpp::ParameterType::PARAMETER_STRING, trajectory_frame_param_desc);
  this->declare_parameter(kFreqParam, rclcpp::ParameterType::PARAMETER_DOUBLE, freq_param_desc);
  this->declare_parameter(kDriveModeParam, rclcpp::ParameterType::PARAMETER_BOOL, driveMode_param_desc);
  this->declare_parameter(kNStatesParam, rclcpp::ParameterType::PARAMETER_INTEGER, nStates_param_desc);
  this->declare_parameter(kVRefParam, rclcpp::ParameterType::PARAMETER_DOUBLE, vRef_param_desc);
  this->declare_parameter(kAMaxDecelParam, rclcpp::ParameterType::PARAMETER_DOUBLE, aMaxDecel_param_desc);

  // load parameter
  try {
    trajectory_frame_id_ = this->get_parameter(kTrajectoryFrameParam).as_string();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set. Using default value: %s", kTrajectoryFrameParam.c_str(), trajectory_frame_id_.c_str());
  }
  try {
    freq_ = this->get_parameter(kFreqParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set. Using default value: %f", kFreqParam.c_str(), freq_);
  }
  try {
    drivable_mode_ = this->get_parameter(kDriveModeParam).as_bool();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set. Using default value: %d", kDriveModeParam.c_str(),
                drivable_mode_);
  }
  try {
    n_states_ = this->get_parameter(kNStatesParam).as_int();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set. Using default value: %d", kNStatesParam.c_str(),
                n_states_);
  }
  try {
    v_ref_ = this->get_parameter(kVRefParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set. Using default value: %f", kVRefParam.c_str(), v_ref_);
  }
  try {
    a_max_decel_ = this->get_parameter(kAMaxDecelParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set. Using default value: %f", kAMaxDecelParam.c_str(),
                a_max_decel_);
  }
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
    s_start_break_ = route_.remaining_route.back().z - distance_to_stop_;
    RCLCPP_INFO(this->get_logger(), "Received first route message, start beak s: %f", s_start_break_);
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
    tf = tf2_buffer_->lookupTransform(tra.header.frame_id, tra.header.stamp, route_.header.frame_id,
                                      route_.header.stamp, "map", rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Tranformation is not available: %s", ex.what());
  }
  route_planning_msgs::msg::Route tf_route;
  tf2::doTransform(route_, tf_route, tf);

  // saving remaining route in path and checking if path starts behind trajectory_frame_id_, which could cause unintended behavior for drivable trajectories
  std::vector<geometry_msgs::msg::Point> path = tf_route.remaining_route;
  if (path[0].x < 0.0)
    RCLCPP_WARN(this->get_logger(), "Path starts %f m behind %s. Could cause unintended behavior.", path[0].x, trajectory_frame_id_.c_str());
  if (drivable_mode_) path.insert(path.begin(), geometry_msgs::msg::Point());

  // currently unused - might be useful for publishing drivable trajectories -> only point where ego_data_ is used
  // geometry_msgs::msg::Pose current_pose = perception_msgs::object_access::getPose(ego_data_);
  // double current_velocity = perception_msgs::object_access::getVelocityMagnitude(ego_data_);
  // double current_speed_limit = tf_route.current_speed_limit/3.6;

  // keep maximum the first n_states_ in path (and therefore in trajectory)
  if (n_states_ < path.size()) {
    path.erase(path.begin() + n_states_, path.end());
  }

  // init trajectory and fill with path (route) and velocity (const from param) data
  trajectory_planning_msgs::trajectory_access::initializeTrajectory(tra, type_id, path.size());
  for (size_t i = 0; i < path.size(); i++) {
    double v = v_ref_;
    if (path[i].z >= s_start_break_)
      v = std::sqrt(std::pow(v_ref_, 2) +
                    2 * a_max_decel_ * (path[i].z - s_start_break_));  // decelerate to stop at end of route
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