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

struct SimplePath {
  std_msgs::msg::Header header;
  std::vector<SimplePathPoint> points;
};

class SimplePlannerNode : public rclcpp::Node {
 public:
  SimplePlannerNode();

 private:
  enum InterpolationType { LINEAR = 0, SPLINE = 1 };

  const std::string kEgoDataTopic = "~/ego_data";
  const std::string kRouteTopic = "~/route";
  const std::string kOutputTopic = "~/trajectory";

  /**
   * @brief Declares and loads a ROS parameter
   *
   * @param name name
   * @param param parameter variable to load into
   * @param description description
   * @param add_to_auto_reconfigurable_params enable reconfiguration of parameter
   * @param is_required whether failure to load parameter will stop node
   * @param read_only set parameter to read-only
   * @param from_value parameter range minimum
   * @param to_value parameter range maximum
   * @param step_value parameter range step
   * @param additional_constraints additional constraints description
   */
  template <typename T>
  void declareAndLoadParameter(const std::string &name,
                               T &param,
                               const std::string &description,
                               const bool add_to_auto_reconfigurable_params = true,
                               const bool is_required = false,
                               const bool read_only = false,
                               const std::optional<double> &from_value = std::nullopt,
                               const std::optional<double> &to_value = std::nullopt,
                               const std::optional<double> &step_value = std::nullopt,
                               const std::string &additional_constraints = "");

  /**
   * @brief Handles reconfiguration when a parameter value is changed
   *
   * @param parameters parameters
   * @return parameter change result
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

  /**
   * @brief Sets up subscribers, publishers, etc. to configure the node
   */
  void setup();

  void egoDataCallback(const perception_msgs::msg::EgoData::UniquePtr msg);
  void routeCallback(const route_planning_msgs::msg::Route::UniquePtr msg);

  void publishTimerCallback();

  trajectory_planning_msgs::msg::Trajectory createTrajectory();
  std::vector<SimplePathPoint> resamplePath(const std::vector<SimplePathPoint>& path, bool stop_at_end, double offset_to_stop_line = 0.0);
  std::vector<SimplePathPoint> generateLaneChangePath(const int start_idx, const int turn_idx,
                                                      const route_planning_msgs::msg::Route& route);
  void recalculateS(std::vector<SimplePathPoint>& path);
  SimplePath calculateSafeStopAlongEgoHeading(const perception_msgs::msg::EgoData& ego_data, const double safe_stop_distance, const std_msgs::msg::Header& target_header);
  SimplePath calculateSafeStopAlongRoute(const SimplePath& path, const double safe_stop_distance);
  SimplePath convertRouteToSimplePath(const route_planning_msgs::msg::Route& tf_route);
  SimplePath transformPath(const SimplePath& path, const std_msgs::msg::Header& target_header);

  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  rclcpp::Subscription<perception_msgs::msg::EgoData>::SharedPtr sub_egoData_;
  rclcpp::Subscription<route_planning_msgs::msg::Route>::SharedPtr sub_route_;

  rclcpp::Publisher<trajectory_planning_msgs::msg::Trajectory>::SharedPtr pub_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  /**
   * @brief Auto-reconfigurable parameters for dynamic reconfiguration
   */
  std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter &)>>> auto_reconfigurable_params_;

  /**
   * @brief Callback handle for dynamic parameter reconfiguration
   */
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  // Parameters
  std::string trajectory_frame_id_ = "base_link";
  std::string fixed_over_time_frame_id_ = "map";
  double freq_ = 10.0;
  double route_timeout_ = 1.0;
  double ego_data_timeout_ = 1.0;
  double trajectory_horizon_ = 6.0;
  int n_states_ = 51;
  bool internal_route_update_ = false;
  uint8_t interpolation_type_ = InterpolationType::SPLINE;
  double standstill_threshold_ = 0.2;
  double v_ref_ = 13.89;
  double a_decel_ = -2.5;
  double a_max_decel_ = -3.5;
  bool consider_traffic_lights_ = false;
  double offset_to_stop_line_ = 0.0;
  double ignore_stop_line_threshold_ = 0.1;
  bool consider_future_states_ = false;
  double lane_change_distance_factor_ = 6.0;
  double lane_change_min_distance_factor_ = 2.0;

  perception_msgs::msg::EgoData ego_data_;
  route_planning_msgs::msg::Route route_;

  bool ego_data_init_ = false;
  bool route_init_ = false;
  double safe_stop_distance_ = -1.0;
  SimplePath latest_path_;
  double dt_;
};

}  // namespace simple_planner