#include <math.h>

#include <chrono>
#include <functional>
#include <thread>

#include <simple_planner/simple_planner_node.hpp>



/**
 * @brief Namespace for simple_planner package
 *
 */
namespace simple_planner {


// parameter names

// constants
const std::string SimplePlannerNode::kEgoDataTopic = "~/ego_data_topic";
const std::string SimplePlannerNode::kRouteTopic = "~/route_topic";
const std::string SimplePlannerNode::kOutputTopic = "~/trajectory_topic";
const std::string SimplePlannerNode::kFreqParam = "frequency";


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

  // declare parameter
  this->declare_parameter(kFreqParam, rclcpp::ParameterType::PARAMETER_DOUBLE, freq_param_desc);

  // load parameter
  try {
    freq_ = this->get_parameter(kFreqParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kFreqParam.c_str());
    exit(EXIT_FAILURE);
  }
}


/**
 * @brief Sets up subscribers, publishers, and more.
 *
 */
void SimplePlannerNode::setup() {

  // create subscriber for handling incoming messages
  sub_egoData_ =
    this->create_subscription<perception_interfaces::msg::EgoData>(
      kEgoDataTopic, 10,
      std::bind(&SimplePlannerNode::egoDataCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_egoData_->get_topic_name());
  sub_route_ =
    this->create_subscription<route_planning_interfaces::msg::Route>(
      kRouteTopic, 10,
      std::bind(&SimplePlannerNode::routeCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_route_->get_topic_name());

  // create a publisher for publishing messages
  pub_ = this->create_publisher<trajectory_interfaces::msg::Trajectory>(
    kOutputTopic, 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", pub_->get_topic_name());

  // create a timer for repeatedly invoking a callback to publish messages
  publish_timer_ =
    this->create_wall_timer(std::chrono::duration<double>(1.0/freq_),
                            std::bind(&SimplePlannerNode::publishTimerCallback,
                            this));
}


/**
 * @brief This callback is invoked when the subscriber receives a new egoData message
 *
 * @param[in] msg   egoData
 */
void SimplePlannerNode::egoDataCallback(
  const perception_interfaces::msg::EgoData::UniquePtr msg) {

  ego_data_ = *msg;
  // RCLCPP_INFO(this->get_logger(), "Received EgoData!");
}

/**
 * @brief This callback is invoked when the subscriber receives a new route message
 * 
 * @param[in] msg   route
 */
void SimplePlannerNode::routeCallback(
  const route_planning_interfaces::msg::Route::UniquePtr msg) {

  route_ = *msg;
  // RCLCPP_INFO(this->get_logger(), "Received Route!");
}

trajectory_interfaces::msg::Trajectory SimplePlannerNode::createTrajectory() {
  std::vector<geometry_msgs::msg::Point> path = route_.shortest_path;
  while(path[0].x < 0.0) {
    path.erase(path.begin());
  }
  geometry_msgs::msg::Pose current_pose = perception_interfaces::object_access::getPose(ego_data_);
  double current_velocity = perception_interfaces::object_access::getVelocityMagnitude(ego_data_);
  double current_speed_limit = route_.current_speed_limit/3.6;

  trajectory_interfaces::msg::Trajectory tra;
  trajectory_interfaces::trajectory_access::initializeTrajectory(tra, trajectory_interfaces::DRIVABLE::TYPE_ID, path.size());
  tra.header.stamp = now();
  tra.header.frame_id = "base_link";

  for (int i = 0; i < path.size(); i++) {
    // set the state
    
    trajectory_interfaces::trajectory_access::setX(tra, path[i].x, i);
    trajectory_interfaces::trajectory_access::setY(tra, path[i].y, i);
    trajectory_interfaces::trajectory_access::setV(tra, current_speed_limit, i);
    trajectory_interfaces::trajectory_access::setS(tra, calcDistance(path, i), i);
    trajectory_interfaces::trajectory_access::setT(tra, calcDistance(path, i)/current_speed_limit, i);
  }

  trajectory_interfaces::trajectory_access::setStandstill(tra, isDestinationReached(current_pose, route_.target_position));
  return tra;
}

bool SimplePlannerNode::isDestinationReached(const geometry_msgs::msg::Pose& current_pose, const geometry_msgs::msg::Point& destination) {
  double distance = sqrt(pow(current_pose.position.x - destination.x, 2) + pow(current_pose.position.y - destination.y, 2));
  return distance < 0.5;
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

/**
 * @brief This callback is invoked every period seconds by the timer
 *
 */
void SimplePlannerNode::publishTimerCallback() {
  trajectory_interfaces::msg::Trajectory msg = createTrajectory();

  pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Published Trajectory!");
}


}


int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_planner::SimplePlannerNode>());
  rclcpp::shutdown();

  return 0;
}
