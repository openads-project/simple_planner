#include <math.h>

#include <chrono>
#include <functional>
#include <thread>

#include <simple_planner/simple_controller_node.hpp>



/**
 * @brief Namespace for simple_planner package
 *
 */
namespace simple_controller {


// parameter names

// constants
const std::string SimpleControllerNode::kEgoDataTopic = "~/ego_data_topic";
const std::string SimpleControllerNode::kTrajectoryTopic = "~/trajectory_topic";
const std::string SimpleControllerNode::kOutputPose = "/carla/ego_vehicle/control/set_transform";
const std::string SimpleControllerNode::kOutputTwist = "/carla/ego_vehicle/control/set_target_velocity";
const std::string SimpleControllerNode::kOutputCtrl = "/carla/ego_vehicle/vehicle_control_cmd";


/**
 * @brief Creates a SimpleControllerNode node
 *
 */
SimpleControllerNode::SimpleControllerNode() : Node("simple_controller_node") {

  this->setup();
}


/**
 * @brief Sets up subscribers, publishers, and more.
 *
 */
void SimpleControllerNode::setup() {

  // tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  // tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // create subscriber for egoData
  sub_egoData_ =
    this->create_subscription<perception_interfaces::msg::EgoData>(
      kEgoDataTopic, 10,
      std::bind(&SimpleControllerNode::egoDataCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_egoData_->get_topic_name());

  // create subscriber for Trajectory
  sub_trajectory_ =
    this->create_subscription<trajectory_interfaces::msg::Trajectory>(
      kTrajectoryTopic, 10,
      std::bind(&SimpleControllerNode::trajectoryCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_trajectory_->get_topic_name());

  // create publisher for Pose and Twist (pass to carla topics)
  // pub_pose_ = this->create_publisher<geometry_msgs::msg::Pose>(kOutputPose, 10);
  // RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", pub_pose_->get_topic_name());
  // pub_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(kOutputTwist, 10);
  // RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", pub_twist_->get_topic_name());

  // create publisher for Ego Vehicle Control message (pass to carla topic)
  pub_ctrl_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(kOutputCtrl, 10);

  pub_duration_ = 1.0/100.0;

  publish_timer_ =
    this->create_wall_timer(std::chrono::duration<double>(pub_duration_),
                            std::bind(&SimpleControllerNode::publishTimerCallback,
                            this));

}


/**
 * @brief This callback is invoked when the subscriber receives a new egoData message
 *
 * @param[in] msg   egoData
 */
void SimpleControllerNode::egoDataCallback(
  const perception_interfaces::msg::EgoData::UniquePtr msg) {

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
  const trajectory_interfaces::msg::Trajectory::UniquePtr msg) {

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

void SimpleControllerNode::trajectoryToCarlaCtrl(const trajectory_interfaces::msg::Trajectory tra) {

  double des_time = (now() - tra.header.stamp).seconds();
  double v_tgt;
  double x_tgt;
  double y_tgt;
  double theta_tgt;

  // Derive State Vectors
  std::vector<double> TIME, V, X, Y, THETA;
  int n_samples = trajectory_interfaces::trajectory_access::getSamplePointSize(tra);
  for(int i=0; i<n_samples; i++){
    TIME.push_back(trajectory_interfaces::trajectory_access::getT(tra, i));
    V.push_back(trajectory_interfaces::trajectory_access::getV(tra, i));
    X.push_back(trajectory_interfaces::trajectory_access::getX(tra, i));
    Y.push_back(trajectory_interfaces::trajectory_access::getY(tra, i));
    THETA.push_back(trajectory_interfaces::trajectory_access::getTheta(tra, i));
  }

  // Interpolate target states by time
  if(!linearInterpolation(TIME, V, des_time, v_tgt)) return;
  if(!linearInterpolation(TIME, X, des_time, x_tgt)) return;
  if(!linearInterpolation(TIME, Y, des_time, y_tgt)) return;
  if(!linearInterpolation(TIME, THETA, des_time, theta_tgt)) return;


  // Implementation to publish steering angle and throttle/brake messages
  carla_msgs::msg::CarlaEgoVehicleControl ctrl_msg;

  // set header
  ctrl_msg.header.stamp = trajectory_.header.stamp;
  ctrl_msg.header.frame_id = "base_link";

  // fetch current ego pose
  geometry_msgs::msg::Pose current_pose = perception_interfaces::object_access::getPose(ego_data_);

  // set longitudinal control (throttle and brake)
  double long_output = longitudinalControlStep(perception_interfaces::object_access::getVelocityMagnitude(ego_data_), v_tgt);

  if (long_output >= 0.0){
    ctrl_msg.throttle = long_output;
    ctrl_msg.brake = 0.0;
  }
  else {
    ctrl_msg.throttle = 0.0;
    ctrl_msg.brake = (-1) * long_output;
  }

  // set lateral control (steering angle)
  double target_yaw = std::atan2(y_tgt, x_tgt); // Yaw angle to get to the target from current ego pose, NOT yaw angle of target pose!
  double lat_output = lateralControlStep(0.0, target_yaw);
  ctrl_msg.steer = -lat_output;

  // set other states that are not relevant
  ctrl_msg.hand_brake = false;
  ctrl_msg.reverse = false;
  ctrl_msg.manual_gear_shift = false;

  // publish control message
  pub_ctrl_->publish(ctrl_msg);


  // Implementation to publish pose and twist messages
  
  // // Wrap interpolations into pose
  // geometry_msgs::msg::PoseStamped pose_bl;
  // pose_bl.header.stamp = trajectory_.header.stamp;
  // pose_bl.header.frame_id = "base_link";
  // pose_bl.pose.position.x = x_tgt;
  // pose_bl.pose.position.y = y_tgt;

  // // Set yaw for target pose (in base_link frame)
  // tf2::Quaternion quat_tf;
  // quat_tf.setRPY(0, 0, theta_tgt);
  // pose_bl.pose.orientation = tf2::toMsg(quat_tf);

  // // Get transform from base_link to carla_map frame
  // auto timeout = rclcpp::Duration::from_seconds(1.0);
  // geometry_msgs::msg::TransformStamped base_link_to_carla_map_tf;
  // try {
  //   base_link_to_carla_map_tf = tf2_buffer_->lookupTransform("carla_map", pose_bl.header.frame_id, pose_bl.header.stamp, timeout);
  // } catch (tf2::TransformException& ex) {
  //   RCLCPP_WARN(this->get_logger(), "Tranformation from %s to 'carla_map' is not available", pose_bl.header.frame_id.c_str());
  //   return;
  // }

  // // Transform pose from base_link to carla_map frame
  // geometry_msgs::msg::PoseStamped pose_map;
  // tf2::doTransform(pose_bl, pose_map, base_link_to_carla_map_tf);

  // // Set velocity of pose by backward differences
  // geometry_msgs::msg::Twist twist;
  // if (!recent_pose_init_) {
  //   // If no recent pose exists, set velocity to zero (only for very first pose)
  //   twist.linear.x = 0;
  //   twist.linear.y = 0;
  //   twist.linear.z = 0;
  //   twist.angular.x = 0;
  //   twist.angular.y = 0;
  //   twist.angular.z = 0;
  //   recent_pose_init_ = true;
  // }
  // else {
  //   // Take backwards differences to derive estimated velocites (only set v_x, v_y and yaw rate)
  //   twist.linear.x = (pose_map.pose.position.x - recent_pose_.position.x) / pub_duration_; // v_x
  //   twist.linear.y = (pose_map.pose.position.y - recent_pose_.position.y) / pub_duration_; // v_y
  //   twist.linear.z = 0;

  //   twist.angular.x = 0;
  //   twist.angular.y = 0;
  //   double current_roll, current_pitch, current_yaw, recent_roll, recent_pitch, recent_yaw;
  //   tf2::Quaternion q_current(pose_map.pose.orientation.x, pose_map.pose.orientation.y, pose_map.pose.orientation.z, pose_map.pose.orientation.w);
  //   tf2::Matrix3x3 m_current(q_current);
  //   m_current.getRPY(current_roll, current_pitch, current_yaw);
  //   tf2::Quaternion q_recent(recent_pose_.orientation.x, recent_pose_.orientation.y, recent_pose_.orientation.z, recent_pose_.orientation.w);
  //   tf2::Matrix3x3 m_recent(q_recent);
  //   m_recent.getRPY(recent_roll, recent_pitch, recent_yaw);
  //   twist.angular.z = (current_yaw - recent_yaw) / pub_duration_ ; // yaw rate
  // }  

  // // Publish pose and twist to carla
  // pub_pose_->publish(pose_map.pose);
  // recent_pose_ = pose_map.pose;
  // pub_twist_->publish(twist);
  // RCLCPP_DEBUG(this->get_logger(), "Published pose and twist to Carla!");
}

double SimpleControllerNode::longitudinalControlStep(double current_velocity, double target_velocity)
{
  double previous_error = error_long;
  error_long = target_velocity - current_velocity;
  // restrict integral term to avoid integral windup
  error_long_integral = std::max(-40.0, std::min(error_long_integral + error_long, 40.0));
  error_long_derivative = error_long - previous_error;
  double output = p_long * error_long + i_long * error_long_integral + d_long * error_long_derivative;
  return std::max(-1.0, std::min(output, 1.0));
}

double SimpleControllerNode::lateralControlStep(double current_yaw, double target_yaw)
{
  double previous_error = error_lat;
  error_lat = target_yaw - current_yaw;
  // restrict integral term to avoid integral windup
  error_lat_integral = std::max(-400.0, std::min(error_lat_integral + error_lat, 400.0));
  error_lat_derivative = error_lat - previous_error;
  double output = p_lat * error_lat + i_lat * error_lat_integral + d_lat * error_lat_derivative;
  return std::max(-1.0, std::min(output, 1.0));

}

bool SimpleControllerNode::linearInterpolation(const std::vector<double>& X, const std::vector<double>& Y, const double& desired_x, double& output_y)
{
  if (desired_x < *min_element(X.begin(), X.end()) || desired_x > *max_element(X.begin(), X.end()))
  {
    RCLCPP_ERROR(get_logger(), "Desired Time is not in between of Time-Min and Time-Max of the given vector!");
    RCLCPP_ERROR(get_logger(), "Desired Time: %f s", desired_x);
    RCLCPP_ERROR(get_logger(), "Time-Min: %f s", *min_element(X.begin(), X.end()));
    RCLCPP_ERROR(get_logger(), "Time-Max: %f s", *max_element(X.begin(), X.end()));
    return false;
  }
  if(X.size() != Y.size())
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Input vectors don't have the same length!");
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
