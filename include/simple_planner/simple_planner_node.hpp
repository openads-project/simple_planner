#pragma once

#include <rclcpp/rclcpp.hpp>

#include <perception_interfaces/msg/ego_data.hpp>
#include <perception_interfaces/object_access.hpp>

#include <route_planning_interfaces/msg/route.hpp>

#include <trajectory_interfaces/msg/trajectory.hpp>
#include <trajectory_interfaces/trajectory_access.hpp>

namespace simple_planner {


class SimplePlannerNode : public rclcpp::Node {

 public:

  SimplePlannerNode();

 private:

  static const std::string kEgoDataTopic;
  static const std::string kRouteTopic;
  static const std::string kOutputTopic;
  static const std::string kFreqParam;
  static const std::string kDriveModeParam;

 private:

  void loadParameters();

  void setup();

  void egoDataCallback(const perception_interfaces::msg::EgoData::UniquePtr msg);
  void routeCallback(const route_planning_interfaces::msg::Route::UniquePtr msg);

  trajectory_interfaces::msg::Trajectory createTrajectory();
  trajectory_interfaces::msg::Trajectory createDemoTrajectory();
  bool isDestinationReached(const geometry_msgs::msg::Pose& current_pose, const geometry_msgs::msg::Point& destination);
  double calcDistance(const std::vector<geometry_msgs::msg::Point>& points, const int& nPoint);

  void publishTimerCallback();
  void publishDemoCallback();

 private:

  rclcpp::Subscription<perception_interfaces::msg::EgoData>::SharedPtr sub_egoData_;
  rclcpp::Subscription<route_planning_interfaces::msg::Route>::SharedPtr sub_route_;

  rclcpp::Publisher<trajectory_interfaces::msg::Trajectory>::SharedPtr pub_;
  rclcpp::Publisher<trajectory_interfaces::msg::Trajectory>::SharedPtr pub_demo_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr demo_timer_;

  // Parameters
  double freq_ = 1.0;
  bool drivable_mode_ = false;

  perception_interfaces::msg::EgoData ego_data_;
  route_planning_interfaces::msg::Route route_;

};


}
