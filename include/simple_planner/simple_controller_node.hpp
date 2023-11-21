#pragma once

#include <rclcpp/rclcpp.hpp>

#include <perception_interfaces/msg/ego_data.hpp>
#include <perception_interfaces/object_access.hpp>

#include <trajectory_interfaces/msg/trajectory.hpp>
#include <trajectory_interfaces/trajectory_access.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace simple_controller {


class SimpleControllerNode : public rclcpp::Node {

 public:

  SimpleControllerNode();

 private:

  static const std::string kEgoDataTopic;
  static const std::string kTrajectoryTopic;
  static const std::string kOutputPose;
  static const std::string kOutputTwist;
  static const std::string kOutputCtrl;

 private:

  void setup();

  void egoDataCallback(const perception_interfaces::msg::EgoData::UniquePtr msg);
  void trajectoryCallback(const trajectory_interfaces::msg::Trajectory::UniquePtr msg);
  void publishTimerCallback();

  double longitudinalControlStep(double current_velocity, double target_velocity);
  double lateralControlStep(double current_yaw, double target_yaw);

  void trajectoryToCarlaCtrl(trajectory_interfaces::msg::Trajectory tra);
  bool linearInterpolation(const std::vector<double>& X, const std::vector<double>& Y, const double& desired_x, double& output_y);

 private:

  // std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  // std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  rclcpp::Subscription<perception_interfaces::msg::EgoData>::SharedPtr sub_egoData_;
  rclcpp::Subscription<trajectory_interfaces::msg::Trajectory>::SharedPtr sub_trajectory_;

  // rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_pose_;
  // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;

  rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr pub_ctrl_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  perception_interfaces::msg::EgoData ego_data_;
  trajectory_interfaces::msg::Trajectory trajectory_;

  double pub_duration_;
  // geometry_msgs::msg::Pose recent_pose_;
  
  bool ego_data_init_ = false;
  bool trajectory_init_ = false;
  // bool recent_pose_init_ = false; 


  // PID Controller
  
  // Longitudinal Controller Parameters:
  double p_long = 0.206;
  double i_long = 0.0206;
  double d_long = 0.515;

  // Lateral Controller Parameters:
  double p_lat = 0.9;
  double i_lat = 0.0;
  double d_lat = 0.0;

  // PID Controller state variables
  double error_long = 0.0;
  double error_long_integral = 0.0;
  double error_long_derivative = 0.0;
  double error_lat = 0.0;
  double error_lat_integral = 0.0;
  double error_lat_derivative = 0.0;

};


}
