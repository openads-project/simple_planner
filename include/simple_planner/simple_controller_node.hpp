#pragma once

#include <rclcpp/rclcpp.hpp>

#include <perception_interfaces/msg/ego_data.hpp>
#include <perception_interfaces/object_access.hpp>

#include <trajectory_interfaces/msg/trajectory.hpp>
#include <trajectory_interfaces/trajectory_access.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace simple_controller {


class SimpleControllerNode : public rclcpp::Node {

 public:

  SimpleControllerNode();

 private:

  static const std::string kEgoDataTopic;
  static const std::string kTrajectoryTopic;
  static const std::string kOutputPose;
  static const std::string kOutputTwist;

 private:

  void setup();

  void egoDataCallback(const perception_interfaces::msg::EgoData::UniquePtr msg);
  void trajectoryCallback(const trajectory_interfaces::msg::Trajectory::UniquePtr msg);
  void publishTimerCallback();

  void trajectoryToCarlaCtrl(trajectory_interfaces::msg::Trajectory tra);
  bool linearInterpolation(const std::vector<double>& X, const std::vector<double>& Y, const double& desired_x, double& output_y);

 private:

  rclcpp::Subscription<perception_interfaces::msg::EgoData>::SharedPtr sub_egoData_;
  rclcpp::Subscription<trajectory_interfaces::msg::Trajectory>::SharedPtr sub_trajectory_;

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  perception_interfaces::msg::EgoData ego_data_;
  trajectory_interfaces::msg::Trajectory trajectory_;

  bool ego_data_init_ = false;
  bool trajectory_init_ = false;
};


}
