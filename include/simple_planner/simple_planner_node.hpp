#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs_utils/object_access.hpp>

#include <rclcpp/rclcpp.hpp>

#include <route_planning_msgs/msg/route.hpp>
#include <route_planning_msgs_utils/route_access.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_route_planning_msgs/tf2_route_planning_msgs.hpp>

#include <trajectory_planning_msgs/msg/trajectory.hpp>
#include <trajectory_planning_msgs_utils/trajectory_access.hpp>

namespace simple_planner {

// only required for parameter handling
template <typename C> struct is_vector : std::false_type {};
template <typename T,typename A> struct is_vector< std::vector<T,A> > : std::true_type {};
template <typename C> inline constexpr bool is_vector_v = is_vector<C>::value;

struct SimplePathPoint {
  Eigen::Vector2d position;
  double s;
  double v;

  // custom constructor
  SimplePathPoint(const Eigen::Vector2d& pos, double s = -1.0, double v = -1.0)
  : position(pos), s(s), v(v) {}

  // default constructor
  SimplePathPoint() = default;
};

class SimplePlannerNode : public rclcpp::Node {
 public:
  SimplePlannerNode();

 private:
  enum InterpolationType { LINEAR = 0, SPLINE = 1 };

  const std::string kEgoDataTopic = "~/ego_data";
  const std::string kRouteTopic = "~/route";
  const std::string kOutputTopic = "~/trajectory";

  template <typename T>
  void declareAndLoadParameter(const std::string &name, T &member_param, const std::string &description,
                               const bool add_to_auto_reconfigurable_params = true, const bool is_required = false,
                               const bool read_only = false, const std::optional<T> &from_value = std::nullopt,
                               const std::optional<T> &to_value = std::nullopt,
                               const std::optional<T> &step_value = std::nullopt,
                               const std::string &additional_constraints = "");
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

  void setup();

  void egoDataCallback(const perception_msgs::msg::EgoData::UniquePtr msg);
  void routeCallback(const route_planning_msgs::msg::Route::UniquePtr msg);

  trajectory_planning_msgs::msg::Trajectory createTrajectory();
  std::vector<SimplePathPoint> resamplePath(const std::vector<SimplePathPoint>& path, bool stop_at_end, double offset_to_stop_line = 0.0);

  void publishTimerCallback();

  std::vector<SimplePathPoint> generateLaneChangePath(const int start_idx, const int turn_idx,
                                                      const route_planning_msgs::msg::Route& route);
  void recalculateS(std::vector<SimplePathPoint>& path);

  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  rclcpp::Subscription<perception_msgs::msg::EgoData>::SharedPtr sub_egoData_;
  rclcpp::Subscription<route_planning_msgs::msg::Route>::SharedPtr sub_route_;

  rclcpp::Publisher<trajectory_planning_msgs::msg::Trajectory>::SharedPtr pub_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Parameters
  std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter &)>>> auto_reconfigurable_params_;
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;
  std::string trajectory_frame_id_ = "base_link";
  std::string fixed_over_time_frame_id_ = "map";
  double freq_ = 10.0;
  double route_timeout_ = 1.0;
  double trajectory_horizon_ = 6.0;
  int n_states_ = 51;
  bool internal_route_update_ = false;
  uint8_t interpolation_type_ = InterpolationType::SPLINE;
  double v_ref_ = 13.89;
  double a_max_decel_ = -2.5;
  bool consider_traffic_lights_ = false;
  double offset_to_stop_line_ = 0.0;
  bool lane_change_restriction_ = true;
  double lane_change_distance_factor_ = 6.0;
  double lane_change_min_distance_factor_ = 2.0;

  perception_msgs::msg::EgoData ego_data_;
  route_planning_msgs::msg::Route route_;
  std::vector<double> v_profile_;

  bool ego_data_init_ = false;
  bool route_init_ = false;
  double s_start_brake_ = std::numeric_limits<double>::infinity();
  double distance_to_stop_;
  double dt_;
};

}  // namespace simple_planner