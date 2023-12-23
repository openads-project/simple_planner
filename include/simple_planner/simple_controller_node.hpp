#pragma once

#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs_utils/object_access.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <trajectory_planning_msgs/msg/trajectory.hpp>
#include <trajectory_planning_msgs_utils/trajectory_access.hpp>

namespace simple_controller {


class SimpleControllerNode : public rclcpp::Node {

 public:

  SimpleControllerNode();

 private:

  static const std::string kEgoDataTopic;
  static const std::string kTrajectoryTopic;
  static const std::string kOutputCtrl;
  static const std::string kFreqParam;
  static const std::string kPLongParam;
  static const std::string kILongParam;
  static const std::string kDLongParam;
  static const std::string kPLatParam;
  static const std::string kILatParam;
  static const std::string kDLatParam;
  static const std::string kLookaheadTime;

 private:

  void setup();
  void loadParameters();

  void egoDataCallback(const perception_msgs::msg::EgoData::UniquePtr msg);
  void trajectoryCallback(const trajectory_planning_msgs::msg::Trajectory::UniquePtr msg);
  void publishTimerCallback();

  double longitudinalControlStep(double current_velocity, double target_velocity);
  double lateralControlStep(double current_yaw, double target_yaw);

  void trajectoryToCarlaCtrl(trajectory_planning_msgs::msg::Trajectory tra);
  bool linearInterpolation(const std::vector<double>& X, const std::vector<double>& Y, const double& desired_x, double& output_y);

 private:

  rclcpp::Subscription<perception_msgs::msg::EgoData>::SharedPtr sub_egoData_;
  rclcpp::Subscription<trajectory_planning_msgs::msg::Trajectory>::SharedPtr sub_trajectory_;

  rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr pub_ctrl_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  perception_msgs::msg::EgoData ego_data_;
  trajectory_planning_msgs::msg::Trajectory trajectory_;

  double frequency_;
  
  bool ego_data_init_ = false;
  bool trajectory_init_ = false;


  // PID Controller
  
  // Longitudinal Controller Parameters:
  double p_long_;
  double i_long_;
  double d_long_;

  // Lateral Controller Parameters:
  double p_lat_;
  double i_lat_;
  double d_lat_;

  // PID Controller state variables
  double error_long_ = 0.0;
  double error_long_integral_ = 0.0;
  double error_long_derivative_ = 0.0;
  double error_lat_ = 0.0;
  double error_lat_integral_ = 0.0;
  double error_lat_derivative_ = 0.0;

  // PID Controller lookahead time
  double lookahead_time_; // s
};


}
