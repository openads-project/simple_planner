#include <math.h>

#include <chrono>
#include <functional>
#include <thread>

#include <simple_planner/simple_controller_node.hpp>



/**
 * @brief Namespace for simple_controller
 *
 */
namespace simple_controller {


// parameter names

// constants
const std::string SimpleControllerNode::kEgoDataTopic = "~/ego_data_topic";
const std::string SimpleControllerNode::kTrajectoryTopic = "~/trajectory_topic";
const std::string SimpleControllerNode::kOutputCtrl = "/carla/ego_vehicle/vehicle_control_cmd";
const std::string SimpleControllerNode::kFreqParam = "frequency";
const std::string SimpleControllerNode::kPLongParam = "p_longitudinal";
const std::string SimpleControllerNode::kILongParam = "i_longitudinal";
const std::string SimpleControllerNode::kDLongParam = "d_longitudinal";
const std::string SimpleControllerNode::kPLatParam = "p_lateral";
const std::string SimpleControllerNode::kILatParam = "i_lateral";
const std::string SimpleControllerNode::kDLatParam = "d_lateral";
const std::string SimpleControllerNode::kLookaheadTime = "lookahead_time_pid";

/**
 * @brief Creates a SimpleControllerNode node
 *
 */
SimpleControllerNode::SimpleControllerNode() : Node("simple_controller_node") {

  this->loadParameters();
  this->setup();
}


/**
 * @brief Sets up subscribers, publishers, and more.
 *
 */
void SimpleControllerNode::setup() {

  // create subscriber for egoData
  sub_egoData_ =
    this->create_subscription<perception_msgs::msg::EgoData>(
      kEgoDataTopic, 10,
      std::bind(&SimpleControllerNode::egoDataCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_egoData_->get_topic_name());

  // create subscriber for Trajectory
  sub_trajectory_ =
    this->create_subscription<trajectory_planning_msgs::msg::Trajectory>(
      kTrajectoryTopic, 10,
      std::bind(&SimpleControllerNode::trajectoryCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_trajectory_->get_topic_name());

  // create publisher for Ego Vehicle Control message (pass to carla topic)
  pub_ctrl_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(kOutputCtrl, 10);

  publish_timer_ =
    this->create_wall_timer(std::chrono::duration<double>(1.0/frequency_),
                            std::bind(&SimpleControllerNode::publishTimerCallback,
                            this));
  RCLCPP_INFO(this->get_logger(), "Publishing vehicle control messages at '%f' hz", frequency_);
}

/**
 * @brief Loads ROS parameters used in the node.
 *
 */
void SimpleControllerNode::loadParameters() {

  // set parameter descriptions
  rcl_interfaces::msg::ParameterDescriptor freq_param_desc;
  freq_param_desc.description = "frequency of published control actions";
  rcl_interfaces::msg::ParameterDescriptor p_longitudinal_param_desc;
  p_longitudinal_param_desc.description = "P-Factor of the longitudinal PID controller";
  rcl_interfaces::msg::ParameterDescriptor i_longitudinal_param_desc;
  i_longitudinal_param_desc.description = "I-Factor of the longitudinal PID controller";
  rcl_interfaces::msg::ParameterDescriptor d_longitudinal_param_desc;
  d_longitudinal_param_desc.description = "D-Factor of the longitudinal PID controller";
  rcl_interfaces::msg::ParameterDescriptor p_lateral_param_desc;
  p_lateral_param_desc.description = "P-Factor of the lateral PID controller";
  rcl_interfaces::msg::ParameterDescriptor i_lateral_param_desc;
  i_lateral_param_desc.description = "I-Factor of the lateral PID controller";
  rcl_interfaces::msg::ParameterDescriptor d_lateral_param_desc;
  d_lateral_param_desc.description = "D-Factor of the lateral PID controller";
  rcl_interfaces::msg::ParameterDescriptor lookahead_time_param_desc;
  lookahead_time_param_desc.description = "Lookahead time of both PID controllers";

  // declare parameters
  this->declare_parameter(kFreqParam, rclcpp::ParameterType::PARAMETER_DOUBLE, freq_param_desc);
  this->declare_parameter(kPLongParam, rclcpp::ParameterType::PARAMETER_DOUBLE, p_longitudinal_param_desc);
  this->declare_parameter(kILongParam, rclcpp::ParameterType::PARAMETER_DOUBLE, i_longitudinal_param_desc);
  this->declare_parameter(kDLongParam, rclcpp::ParameterType::PARAMETER_DOUBLE, d_longitudinal_param_desc);
  this->declare_parameter(kPLatParam, rclcpp::ParameterType::PARAMETER_DOUBLE, p_lateral_param_desc);
  this->declare_parameter(kILatParam, rclcpp::ParameterType::PARAMETER_DOUBLE, i_lateral_param_desc);
  this->declare_parameter(kDLatParam, rclcpp::ParameterType::PARAMETER_DOUBLE, d_lateral_param_desc);
  this->declare_parameter(kLookaheadTime, rclcpp::ParameterType::PARAMETER_DOUBLE, lookahead_time_param_desc);

  // load parameters
  try {
    frequency_ = this->get_parameter(kFreqParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kFreqParam.c_str());
    exit(EXIT_FAILURE);
  }
  try {
    p_long_ = this->get_parameter(kPLongParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kPLongParam.c_str());
    exit(EXIT_FAILURE);
  }
  try {
    i_long_ = this->get_parameter(kILongParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kILongParam.c_str());
    exit(EXIT_FAILURE);
  }
  try {
    d_long_ = this->get_parameter(kDLongParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kDLongParam.c_str());
    exit(EXIT_FAILURE);
  }
  try {
    p_lat_ = this->get_parameter(kPLatParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kPLatParam.c_str());
    exit(EXIT_FAILURE);
  }
  try {
    i_lat_ = this->get_parameter(kILatParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kILatParam.c_str());
    exit(EXIT_FAILURE);
  }
  try {
    d_lat_ = this->get_parameter(kDLatParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kDLatParam.c_str());
    exit(EXIT_FAILURE);
  }
  try {
    lookahead_time_ = this->get_parameter(kLookaheadTime).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kLookaheadTime.c_str());
    exit(EXIT_FAILURE);
  }
}

/**
 * @brief This callback is invoked when the subscriber receives a new egoData message
 *
 * @param[in] msg   egoData
 */
void SimpleControllerNode::egoDataCallback(
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
 * @param[in] msg   trajectory
 */
void SimpleControllerNode::trajectoryCallback(
  const trajectory_planning_msgs::msg::Trajectory::UniquePtr msg) {

  trajectory_ = *msg;

  if (!trajectory_init_){
    trajectory_init_ = true;
    RCLCPP_INFO(this->get_logger(), "Received first trajectory message, initialized global variable");
  }
}

/**
 * @brief This callback is invoked every period seconds by the timer
 *
 */
void SimpleControllerNode::publishTimerCallback() {
  // if trajectory and ego data are not received, do nothing
  if (!trajectory_init_ || !ego_data_init_) {
    return;
  }
  
  trajectoryToCarlaCtrl(trajectory_);
}

void SimpleControllerNode::trajectoryToCarlaCtrl(const trajectory_planning_msgs::msg::Trajectory tra) {
  if (!trajectory_planning_msgs::trajectory_access::getStandstill(tra)) {

    int n_samples = trajectory_planning_msgs::trajectory_access::getSamplePointSize(tra);

    double lookahead_time_current_step = lookahead_time_;

    while (trajectory_planning_msgs::trajectory_access::getT(tra, n_samples - 1) < ((now() - tra.header.stamp).seconds() + lookahead_time_current_step))
    {
      lookahead_time_current_step = lookahead_time_current_step/2;
      if (lookahead_time_current_step < 0.01){
        break; // Current time exceeds planned trajectory even with no lookahead, interpolation will throw an error
      }
    }

    RCLCPP_DEBUG(this->get_logger(), "Lookahead time: %f ", lookahead_time_current_step);

    double des_time = (now() - tra.header.stamp).seconds() + lookahead_time_current_step;
    double v_tgt;
    double x_tgt;
    double y_tgt;
    double theta_tgt;

    // Derive State Vectors
    std::vector<double> TIME, V, X, Y, THETA;
    for(int i=0; i<n_samples; i++){
      TIME.push_back(trajectory_planning_msgs::trajectory_access::getT(tra, i));
      V.push_back(trajectory_planning_msgs::trajectory_access::getV(tra, i));
      X.push_back(trajectory_planning_msgs::trajectory_access::getX(tra, i));
      Y.push_back(trajectory_planning_msgs::trajectory_access::getY(tra, i));
      THETA.push_back(trajectory_planning_msgs::trajectory_access::getTheta(tra, i));
    }

    // Interpolate target states by time
    if(!linearInterpolation(TIME, V, des_time, v_tgt)) return;
    if(!linearInterpolation(TIME, X, des_time, x_tgt)) return;
    if(!linearInterpolation(TIME, Y, des_time, y_tgt)) return;
    if(!linearInterpolation(TIME, THETA, des_time, theta_tgt)) return;

    // Publish steering angle and throttle/brake messages
    carla_msgs::msg::CarlaEgoVehicleControl ctrl_msg;

    // set header
    ctrl_msg.header.stamp = trajectory_.header.stamp;
    ctrl_msg.header.frame_id = "base_link";

    // fetch current ego pose
    geometry_msgs::msg::Pose current_pose = perception_msgs::object_access::getPose(ego_data_);

    // set longitudinal control (throttle and brake)
    double long_output = longitudinalControlStep(perception_msgs::object_access::getVelocityMagnitude(ego_data_), v_tgt);

    if (long_output >= 0.0){
      ctrl_msg.throttle = long_output;
      ctrl_msg.brake = 0.0;
    }
    else {
      ctrl_msg.throttle = 0.0;
      ctrl_msg.brake = 0.0;
      // ctrl_msg.brake = (-1) * long_output;
    }

    // set lateral control (steering angle)
    double target_yaw = std::atan2(y_tgt, x_tgt); // Yaw angle to get to the target from current ego pose, NOT yaw angle of target pose!
    double lat_output = lateralControlStep(0.0, target_yaw);
    ctrl_msg.steer = -lat_output;

    RCLCPP_DEBUG(this->get_logger(), "Computed longitudinal and lateral control:   throttle: %f    brake: %f    steering: %f ", ctrl_msg.throttle, ctrl_msg.brake, ctrl_msg.steer);

    // set other states that are not relevant
    ctrl_msg.hand_brake = false;
    ctrl_msg.reverse = false;
    ctrl_msg.manual_gear_shift = false;

    // publish control message
    pub_ctrl_->publish(ctrl_msg);
  }
  else {
    // reached final destination (standstill = true)
    carla_msgs::msg::CarlaEgoVehicleControl ctrl_msg;
    ctrl_msg.header.stamp = trajectory_.header.stamp;
    ctrl_msg.header.frame_id = "base_link";
    ctrl_msg.throttle = 0.0;
    ctrl_msg.brake = 1.0;
    ctrl_msg.steer = 0.0;
    ctrl_msg.hand_brake = false;
    ctrl_msg.reverse = false;
    ctrl_msg.manual_gear_shift = false;
    pub_ctrl_->publish(ctrl_msg);
  }
}

double SimpleControllerNode::longitudinalControlStep(double current_velocity, double target_velocity)
{
  double previous_error = error_long_;
  error_long_ = target_velocity - current_velocity;
  RCLCPP_WARN(get_logger(), "LongControl: currentVel: %f     targetVel: %f     currentError: %f    prevError: %f ", current_velocity, target_velocity, error_long_, previous_error);
  // restrict integral term to avoid integral windup
  error_long_integral_ = std::max(-40.0, std::min(error_long_integral_ + error_long_, 40.0));
  error_long_derivative_ = error_long_ - previous_error;
  double output = p_long_ * error_long_ + i_long_ * error_long_integral_ + d_long_ * error_long_derivative_;
  return std::max(-1.0, std::min(output, 1.0));
}

double SimpleControllerNode::lateralControlStep(double current_yaw, double target_yaw)
{
  double previous_error = error_lat_;
  error_lat_ = target_yaw - current_yaw;
  // restrict integral term to avoid integral windup
  error_lat_integral_ = std::max(-400.0, std::min(error_lat_integral_ + error_lat_, 400.0));
  error_lat_derivative_ = error_lat_ - previous_error;
  double output = p_lat_ * error_lat_ + i_lat_ * error_lat_integral_ + d_lat_ * error_lat_derivative_;
  return std::max(-1.0, std::min(output, 1.0));
}

bool SimpleControllerNode::linearInterpolation(const std::vector<double>& X, const std::vector<double>& Y, const double& desired_x, double& output_y)
{
  if (desired_x < *min_element(X.begin(), X.end()) || desired_x > *max_element(X.begin(), X.end()))
  {
    RCLCPP_ERROR(get_logger(), "Desired Time is not in between of Time-Min and Time-Max of the given vector!");
    RCLCPP_DEBUG(get_logger(), "Desired Time: %f s", desired_x);
    RCLCPP_DEBUG(get_logger(), "Time-Min: %f s", *min_element(X.begin(), X.end()));
    RCLCPP_DEBUG(get_logger(), "Time-Max: %f s", *max_element(X.begin(), X.end()));
    return false;
  }
  if(X.size() != Y.size())
  {
    RCLCPP_ERROR(get_logger(), "Input vectors don't have the same length!");
    return false;
  }

  //go through array and search for sampling points
  size_t i;
  for(i = 0; i < X.size(); i++)
  {
    if (X[i] < desired_x)
    {
      continue;
    }
    else if (X[i] == desired_x)
    {
      output_y = Y[i];
      return true;
    }
    else
    {
      break;
    }
  }
  output_y = Y[i - 1] + ((Y[i] - Y[i - 1]) / (X[i] - X[i - 1])) * (desired_x - X[i - 1]);
  return true;
}

}


int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_controller::SimpleControllerNode>());
  rclcpp::shutdown();

  return 0;
}
