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
const std::string SimplePlannerNode::kInputTopic = "~/input_topic";
const std::string SimplePlannerNode::kOutputTopic = "~/output_topic";
const std::string SimplePlannerNode::kParam = "param";


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
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = "TODO";

  // set allowed parameter range
  rcl_interfaces::msg::FloatingPointRange param_range;
  param_range.set__from_value(0.1).set__to_value(10.0).set__step(0.1);
  param_desc.floating_point_range = {param_range};

  // declare parameter
  this->declare_parameter(kParam, rclcpp::ParameterType::PARAMETER_DOUBLE, param_desc);

  // load parameter
  try {
    param_ = this->get_parameter(kParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kParam.c_str());
    exit(EXIT_FAILURE);
  }
}


/**
 * @brief Sets up subscribers, publishers, and more.
 *
 */
void SimplePlannerNode::setup() {

  // create a callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(
    std::bind(&SimplePlannerNode::parametersCallback, this, std::placeholders::_1));

  // create a subscriber for handling incoming messages
  subscriber_ =
    this->create_subscription<std_msgs::msg::Int32>(
      kInputTopic, 10,
      std::bind(&SimplePlannerNode::topicCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", subscriber_->get_topic_name());

  // create a publisher for publishing messages
  publisher_ = this->create_publisher<std_msgs::msg::Int32>(
    kOutputTopic, 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", publisher_->get_topic_name());



  // create a timer for repeatedly invoking a callback to publish messages
  publish_timer_ =
    this->create_wall_timer(std::chrono::duration<double>(param_),
                            std::bind(&SimplePlannerNode::publishTimerCallback,
                            this));
}


/**
 * @brief This callback is invoked when a parameter value has changed
 *
 * @param[in] parameters                                  input
 *
 * @return    rcl_interfaces::msg::SetParametersResult    output
 */
rcl_interfaces::msg::SetParametersResult SimplePlannerNode::parametersCallback(
  const std::vector<rclcpp::Parameter> &parameters) {

  // update timer with newly configured period parameter value
  rcl_interfaces::msg::SetParametersResult result;
  for (const auto &param : parameters) {
    if (param.get_name() == kParam) {
      param_ = param.as_double();
    }
  }

  // mark parameter change successful
  result.successful = true;
  result.reason = "success";

  return result;
}


/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input
 */
void SimplePlannerNode::topicCallback(
  const std_msgs::msg::Int32 &msg) {

  RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg.data);
}






/**
 * @brief This callback is invoked every period seconds by the timer
 *
 */
void SimplePlannerNode::publishTimerCallback() {

}


}


int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_planner::SimplePlannerNode>());
  rclcpp::shutdown();

  return 0;
}
