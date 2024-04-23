#include <chrono>
#include <functional>
#include <math.h>
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
const std::string SimplePlannerNode::kDemoTopic = "~/demo_trajectory";
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
  this->declare_parameter(kFreqParam, rclcpp::ParameterType::PARAMETER_DOUBLE, freq_param_desc);
  this->declare_parameter(kDriveModeParam, rclcpp::ParameterType::PARAMETER_BOOL, driveMode_param_desc);
  this->declare_parameter(kNStatesParam, rclcpp::ParameterType::PARAMETER_INTEGER, nStates_param_desc);
  this->declare_parameter(kVRefParam, rclcpp::ParameterType::PARAMETER_DOUBLE, vRef_param_desc);
  this->declare_parameter(kAMaxDecelParam, rclcpp::ParameterType::PARAMETER_DOUBLE, aMaxDecel_param_desc);

  // load parameter
  try {
    freq_ = this->get_parameter(kFreqParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kFreqParam.c_str());
    exit(EXIT_FAILURE);
  }
  try {
    drivable_mode_ = this->get_parameter(kDriveModeParam).as_bool();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kDriveModeParam.c_str());
    exit(EXIT_FAILURE);
  }
  try {
    n_states_ = this->get_parameter(kNStatesParam).as_int();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kNStatesParam.c_str());
    exit(EXIT_FAILURE);
  }
  try {
    v_ref_ = this->get_parameter(kVRefParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kVRefParam.c_str());
    exit(EXIT_FAILURE);
  }
  try {
    a_max_decel_ = this->get_parameter(kAMaxDecelParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kAMaxDecelParam.c_str());
    exit(EXIT_FAILURE);
  }
}

/**
 * @brief Sets up subscribers, publishers, and more.
 *
 */
void SimplePlannerNode::setup() {

  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // create subscriber for egoData
  sub_egoData_ =
    this->create_subscription<perception_msgs::msg::EgoData>(
      kEgoDataTopic, 10,
      std::bind(&SimplePlannerNode::egoDataCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_egoData_->get_topic_name());
  
  // create subscriber for route
  sub_route_ =
    this->create_subscription<route_planning_msgs::msg::Route>(
      kRouteTopic, 10,
      std::bind(&SimplePlannerNode::routeCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_route_->get_topic_name());

  // create a publisher for publishing output trajectory
  pub_ = this->create_publisher<trajectory_planning_msgs::msg::Trajectory>(
    kOutputTopic, 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", pub_->get_topic_name());

  // create a publisher for demo trajectory
  pub_demo_ = this->create_publisher<trajectory_planning_msgs::msg::Trajectory>(
    kDemoTopic, 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", pub_demo_->get_topic_name());

  // create a timer for repeatedly invoking a callback to publish messages
  publish_timer_ =
    this->create_wall_timer(std::chrono::duration<double>(1.0/freq_),
                            std::bind(&SimplePlannerNode::publishTimerCallback,
                            this));
  RCLCPP_INFO(this->get_logger(), "Publishing trajectory at '%f' hz", freq_);

  // create a timer for repeatedly invoking a callback to publish messages
  demo_timer_ =
    this->create_wall_timer(std::chrono::duration<double>(10.0),
                            std::bind(&SimplePlannerNode::publishDemoCallback,
                            this));
  RCLCPP_INFO(this->get_logger(), "Publishing Demo Trajectory at 0.1 hz");
}


/**
 * @brief This callback is invoked when the subscriber receives a new egoData message
 *
 * @param[in] msg   egoData
 */
void SimplePlannerNode::egoDataCallback(
  const perception_msgs::msg::EgoData::UniquePtr msg) {

  ego_data_ = *msg;
  
  if (!ego_data_init_){
    ego_data_init_ = true;
    RCLCPP_INFO(this->get_logger(), "Received first ego data message, initialized global variable");
  }
}

/**
 * @brief This callback is invoked when the subscriber receives a new route message
 * 
 * @param[in] msg   route
 */
void SimplePlannerNode::routeCallback(
  const route_planning_msgs::msg::Route::UniquePtr msg) {

  route_ = *msg;

  if (!route_init_){
    route_init_ = true;
    RCLCPP_INFO(this->get_logger(), "Received first route message, initialized global variable");
  }
}

trajectory_planning_msgs::msg::Trajectory SimplePlannerNode::createDemoTrajectory() {

  trajectory_planning_msgs::msg::Trajectory tra;
  trajectory_planning_msgs::trajectory_access::initializeTrajectory(tra, trajectory_planning_msgs::DRIVABLE::TYPE_ID, 5);
  tra.header.stamp = now();
  tra.header.frame_id = "base_link";

  // const radius

  // t,x,y,v,theta,a,kappa,dkappa,s
  // 0,0,0,0,0,0,0,0,0
  // t,10,0,3,0,0,0,0,10
  // t,45.3553,-14.6447,3,-0.7854,0,0.02,0,49.27
  // t,60,-50,3,-1.5708,0,0.02,0,88.54
  // t,60,-60,3,-1.5708,0,0,0,98.54

  std::vector<double> state0 = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  trajectory_planning_msgs::trajectory_access::setState(tra, state0, 0);
  std::vector<double> state1 = {10.0/3.0,10.0,0.0,3.0,0.0,0.0,0.0,0.0,10.0};
  trajectory_planning_msgs::trajectory_access::setState(tra, state1, 1);
  std::vector<double> state2 = {49.27/3.0,45.3553,-14.6447,3.0,-0.7854,0.0,0.02,0.02,49.27};
  trajectory_planning_msgs::trajectory_access::setState(tra, state2, 2);
  std::vector<double> state3 = {88.54/3.0,60.0,-50.0,3.0,-1.5708,0.0,0.02,0.02,88.54};
  trajectory_planning_msgs::trajectory_access::setState(tra, state3, 3);
  std::vector<double> state4 = {98.54/3.0,60.0,-60.0,3.0,-1.5708,0.0,0.0,0.0,98.54};
  trajectory_planning_msgs::trajectory_access::setState(tra, state4, 4);


  trajectory_planning_msgs::trajectory_access::setStandstill(tra, false);
  return tra;
}

trajectory_planning_msgs::msg::Trajectory SimplePlannerNode::createTrajectory() {

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf2_buffer_->lookupTransform("base_link", now(), "base_link", route_.header.stamp, "map", rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Tranformation is not available: %s", ex.what());
  }
  route_planning_msgs::msg::Route route;
  tf2::doTransform(route_, route, tf);
  std::vector<geometry_msgs::msg::Point> path = route.shortest_path;
  bool validPath = true;
  while(path[0].x < 0.0) {
    path.erase(path.begin());
    if (path.size() <= 1){
      path.erase(path.begin(), path.end());
      // push back three empty points to fill with zeros later
      path.push_back(geometry_msgs::msg::Point());
      path.push_back(geometry_msgs::msg::Point());
      path.push_back(geometry_msgs::msg::Point());
      validPath = false;
      break;
    }
  }
  if (drivable_mode_) path.insert(path.begin(), geometry_msgs::msg::Point());
  geometry_msgs::msg::Pose current_pose = perception_msgs::object_access::getPose(ego_data_);
  double current_velocity = perception_msgs::object_access::getVelocityMagnitude(ego_data_);
  double current_speed_limit = route.current_speed_limit/3.6;

  int n_states_min = n_states_ < path.size() ? n_states_ : path.size();
  int start_break_index = -1;
  double distance_last_point_to_target = sqrt(pow(path.back().x - route.target_position.x, 2) + pow(path.back().y - route.target_position.y, 2));
  //TODO: get from route
  if (distance_last_point_to_target < 1.0) {
    RCLCPP_WARN(this->get_logger(), "Distance to target: %f", distance_last_point_to_target);
    double distance_to_stop = -0.5*pow(v_ref_, 2)/a_max_decel_;
    // get the index of the last point, which distance to the target position is less than distance_to_stop
    double distance_to_target = 0.0;
    for (size_t i = path.size()-1; i > 0; i--) {
      distance_to_target += sqrt(pow(path[i].x - path[i-1].x, 2) + pow(path[i].y - path[i-1].y, 2));
      if (distance_to_target > distance_to_stop) {
        start_break_index = i;
        break;
      } 
    }
  }

  trajectory_planning_msgs::msg::Trajectory tra;
  if (!validPath){
    int type_id = drivable_mode_ ? trajectory_planning_msgs::DRIVABLE::TYPE_ID : trajectory_planning_msgs::REFERENCE::TYPE_ID;
    trajectory_planning_msgs::trajectory_access::initializeTrajectory(tra, type_id, n_states_min);
    tra.header.stamp = now();
    tra.header.frame_id = "base_link";
    for (int i = 0; i < n_states_min; i++) {
      RCLCPP_DEBUG(this->get_logger(), "Invalid Path. i: %ld", i);
      trajectory_planning_msgs::trajectory_access::setT(tra, (double)i, i);
      trajectory_planning_msgs::trajectory_access::setX(tra, 0.0, i);
      trajectory_planning_msgs::trajectory_access::setY(tra, 0.0, i);
      trajectory_planning_msgs::trajectory_access::setV(tra, 0.0, i);
      if (drivable_mode_) {
        trajectory_planning_msgs::trajectory_access::setS(tra, 0.0, i);
        trajectory_planning_msgs::trajectory_access::setTheta(tra, 0.0, i);
        // TODO: setA, setKappa, setDkappa
      }
    }
    trajectory_planning_msgs::trajectory_access::setStandstill(tra, true);
  }
  else {
    int type_id = drivable_mode_ ? trajectory_planning_msgs::DRIVABLE::TYPE_ID : trajectory_planning_msgs::REFERENCE::TYPE_ID;
    trajectory_planning_msgs::trajectory_access::initializeTrajectory(tra, type_id, n_states_min);
    tra.header.stamp = now();
    tra.header.frame_id = "base_link";
    for (int i = 0; i < n_states_min; i++) {
      RCLCPP_DEBUG(this->get_logger(), "Debug: i: %ld,  t: %f,  x: %f,  y: %f,  s: %f,  theta: %f", i, calcDistance(path, i)/v_ref_, path[i].x, path[i].y, calcDistance(path, i), calcTheta(path, i));
      trajectory_planning_msgs::trajectory_access::setT(tra, calcDistance(path, i)/v_ref_, i);
      trajectory_planning_msgs::trajectory_access::setX(tra, path[i].x, i);
      trajectory_planning_msgs::trajectory_access::setY(tra, path[i].y, i);
      trajectory_planning_msgs::trajectory_access::setV(tra, v_ref_, i);
      if (drivable_mode_) {
        trajectory_planning_msgs::trajectory_access::setS(tra, calcDistance(path, i), i);
        trajectory_planning_msgs::trajectory_access::setTheta(tra, calcTheta(path, i), i);
        // TODO: setA, setKappa, setDkappa
      }
    }
    trajectory_planning_msgs::trajectory_access::setStandstill(tra, isDestinationReached(route.target_position));
  }

  if (start_break_index >= 0) {
    // calc number of points to stop between start_break_index and path.size()
    RCLCPP_INFO(this->get_logger(), "Start Break Index: %d", start_break_index);
    double n_points_to_stop = n_states_min - start_break_index;
    RCLCPP_WARN(this->get_logger(), "n_points_to_stop: %f", n_points_to_stop);
    double iter = 1.0;
    for (int i = start_break_index; i < n_states_min; i++) {
      double velocity = v_ref_ - iter/n_points_to_stop * v_ref_;
      RCLCPP_INFO(this->get_logger(), "Velocity: %f", velocity);
      trajectory_planning_msgs::trajectory_access::setV(tra, velocity, i);
      iter++;
    }
  }

  RCLCPP_DEBUG(this->get_logger(), "Standstill = %d", tra.standstill);
  return tra;
}

bool SimplePlannerNode::isDestinationReached(const geometry_msgs::msg::Point& destination) {
  double distance = sqrt(pow(destination.x, 2) + pow(destination.y, 2));
  RCLCPP_DEBUG(this->get_logger(), "Distance to goal: %f", distance);
  return distance < 0.2;
}

double SimplePlannerNode::calcDistance(const std::vector<geometry_msgs::msg::Point>& points, const int& nPoint) {
  double distance = 0.0;
  for (int i = 0; i <= nPoint; i++) {
    if (i == 0) {
      distance += sqrt(pow(points[i].x - 0.0, 2) + pow(points[i].y - 0.0, 2));
    } else {
      distance += sqrt(pow(points[i].x - points[i-1].x, 2) + pow(points[i].y - points[i-1].y, 2));
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
      theta += atan2(points[i].y - points[i-1].y, points[i].x - points[i-1].x);
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
  RCLCPP_INFO(this->get_logger(), "Published Trajectory!");
}

void SimplePlannerNode::publishDemoCallback() {

  trajectory_planning_msgs::msg::Trajectory msg = createDemoTrajectory();

  pub_demo_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Published Demo-Trajectory!");
}

}


int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_planner::SimplePlannerNode>());
  rclcpp::shutdown();

  return 0;
}
