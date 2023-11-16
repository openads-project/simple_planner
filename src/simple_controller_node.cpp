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
  pub_pose_ = this->create_publisher<geometry_msgs::msg::Pose>(kOutputPose, 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", pub_pose_->get_topic_name());
  pub_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(kOutputTwist, 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", pub_twist_->get_topic_name());

  publish_timer_ =
    this->create_wall_timer(std::chrono::duration<double>(1.0/100.0),
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
  // if route and ego data are not received, do nothing
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
  
  // fetch current ego pose
  geometry_msgs::msg::Pose pose = perception_interfaces::object_access::getPose(ego_data_);
  double yaw = perception_interfaces::object_access::getYaw(ego_data_);

  // set yaw for target pose
  tf2::Quaternion quat_tf;
  quat_tf.setRPY(0, 0, yaw + theta_tgt);
  pose.orientation = tf2::toMsg(quat_tf);

  // set x and y for target pose
  double dx = (x_tgt * std::cos(yaw) - y_tgt * std::sin(yaw));
  double dy = (x_tgt * std::sin(yaw) + y_tgt * std::cos(yaw));
  pose.position.x = pose.position.x + dx;
  pose.position.y = pose.position.y + dy;

  // set velocity of pose to zero (pose is set sufficiently often)
  geometry_msgs::msg::Twist twist = geometry_msgs::msg::Twist();
  twist.linear.x = 0;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = 0;

  // publish pose and twist to carla
  pub_pose_->publish(pose);
  pub_twist_->publish(twist);
  RCLCPP_INFO(this->get_logger(), "Published pose and twist to Carla!");
}

bool SimpleControllerNode::linearInterpolation(const std::vector<double>& X, const std::vector<double>& Y, const double& desired_x, double& output_y)
{
  if (desired_x < *min_element(X.begin(), X.end()) || desired_x > *max_element(X.begin(), X.end()))
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Desired X-Value is not in between of X-Min and X-Max of the given vector!");
    return false;
  }
  if(X.size() != Y.size())
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Input vectors don't have the same length!");
    return false;
  }

  //go through array and search for sampling points
  int i;
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
