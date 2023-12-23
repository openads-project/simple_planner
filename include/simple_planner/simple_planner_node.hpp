#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs_utils/object_access.hpp>

#include <rclcpp/rclcpp.hpp>

#include <route_planning_msgs/msg/route.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_route_planning_msgs/tf2_route_planning_msgs.hpp>

#include <trajectory_planning_msgs/msg/trajectory.hpp>
#include <trajectory_planning_msgs_utils/trajectory_access.hpp>

namespace simple_planner {


class SimplePlannerNode : public rclcpp::Node {

 public:

  SimplePlannerNode();

 private:

  static const std::string kEgoDataTopic;
  static const std::string kRouteTopic;
  static const std::string kOutputTopic;
  static const std::string kDemoTopic;
  static const std::string kFreqParam;
  static const std::string kDriveModeParam;

 private:

  void loadParameters();

  void setup();

  void egoDataCallback(const perception_msgs::msg::EgoData::UniquePtr msg);
  void routeCallback(const route_planning_msgs::msg::Route::UniquePtr msg);

  trajectory_planning_msgs::msg::Trajectory createTrajectory();
  trajectory_planning_msgs::msg::Trajectory createDemoTrajectory();
  bool isDestinationReached(const geometry_msgs::msg::Point& destination);
  double calcDistance(const std::vector<geometry_msgs::msg::Point>& points, const int& nPoint);
  double calcTheta(const std::vector<geometry_msgs::msg::Point>& points, const int& nPoint);

  void publishTimerCallback();
  void publishDemoCallback();

 private:

  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  rclcpp::Subscription<perception_msgs::msg::EgoData>::SharedPtr sub_egoData_;
  rclcpp::Subscription<route_planning_msgs::msg::Route>::SharedPtr sub_route_;

  rclcpp::Publisher<trajectory_planning_msgs::msg::Trajectory>::SharedPtr pub_;
  rclcpp::Publisher<trajectory_planning_msgs::msg::Trajectory>::SharedPtr pub_demo_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr demo_timer_;

  // Parameters
  double freq_ = 1.0;
  bool drivable_mode_ = false;

  perception_msgs::msg::EgoData ego_data_;
  route_planning_msgs::msg::Route route_;

  bool ego_data_init_ = false;
  bool route_init_ = false;
};


}
