#pragma once

#include <map>
#include <optional>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs/msg/object_list.hpp>
#include <perception_msgs_utils/object_access.hpp>

#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/set_bool.hpp>

#include <route_planning_msgs/msg/route.hpp>
#include <route_planning_msgs_utils/route_access.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_route_planning_msgs/tf2_route_planning_msgs.hpp>

#include <trajectory_planning_msgs/msg/trajectory.hpp>
#include <trajectory_planning_msgs_utils/trajectory_access.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <simple_planner/object_geometry.hpp>

namespace simple_planner {

// only required for parameter handling
template <typename C> struct is_vector : std::false_type {};
template <typename T,typename A> struct is_vector< std::vector<T,A> > : std::true_type {};
template <typename C> inline constexpr bool is_vector_v = is_vector<C>::value;

/**
 * @brief Configuration parameters for topic diagnostics
 */
struct TopicDiagnosticConfig {
  /**
   * @brief Minimum acceptable frequency
   */
  double min_frequency;

  /**
   * @brief Maximum acceptable frequency
   */
  double max_frequency;

  /**
   * @brief Minimum acceptable difference between message timestamp and receipt time (in seconds)
   */
  double min_acceptable_timestamp_delta;

  /**
   * @brief Maximum acceptable difference between message timestamp and receipt time (in seconds)
   */
  double max_acceptable_timestamp_delta;
};

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
  enum class PlannerState { NoPublish, Standstill, SafeStop, FollowRoute };

  struct FollowRoutePlan {
    SimplePath path;
    bool stop_at_end = false;
    std::string reason_to_stop = "";
    double offset_to_stop_line = 0.0;
    uint8_t suggested_turn_signal = route_planning_msgs::msg::LaneElement::SUGGESTED_TURN_SIGNAL_NONE;
  };

  const std::string kEgoDataTopic = "~/ego_data";
  const std::string kObjectListTopic = "~/object_list";
  const std::string kRouteTopic = "~/route";
  const std::string kOutputTopic = "~/trajectory";
  const std::string kObjectInteractionMarkerTopic = "~/object_interaction_markers";

  const std::string kLeftTurnIndicatorSrv = "~/enable_left_turn_indicator";
  const std::string kRightTurnIndicatorSrv = "~/enable_right_turn_indicator";
  const std::string kHazardLightsSrv = "~/enable_hazard_lights";

  // Internal object-handling tuning values (fixed, intentionally not exposed as parameters)
  static constexpr double kObjectCollisionCheckDt = 0.05;  // maximum time step for swept collision checks (s)
  static constexpr double kMinObjectWidth = 0.8;           // minimum object width if dimensions are missing/too small (m)
  static constexpr double kMinObjectLength = 1.2;          // minimum object length if dimensions are missing/too small (m)

  /**
   * @brief Declares a ROS parameter, loads its value and optionally registers it for runtime updates.
   *
   * @tparam T Parameter value type.
   * @param[in] name Parameter name.
   * @param[out] param Member variable to store the parameter value.
   * @param[in] description Human-readable parameter description.
   * @param[in] add_to_auto_reconfigurable_params Whether parameter updates automatically update the member variable.
   * @param[in] is_required Whether the node should fail if the parameter is not set.
   * @param[in] read_only Whether the parameter is exposed as read-only.
   * @param[in] from_value Optional lower bound for numeric parameters.
   * @param[in] to_value Optional upper bound for numeric parameters.
   * @param[in] step_value Optional step size for numeric parameters.
   * @param[in] additional_constraints Additional free-form constraint text for the parameter descriptor.
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
   * @param[in] parameters Requested parameter updates.
   * @return parameter change result
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

  /**
   * @brief Sets up subscribers, publishers, etc. to configure the node
   */
  void setup();

  void egoDataCallback(const perception_msgs::msg::EgoData::UniquePtr msg);
  /**
   * @brief Stores the latest perceived object list including object predictions.
   *
   * @param msg Latest object list message.
   */
  void objectListCallback(const perception_msgs::msg::ObjectList::UniquePtr msg);
  void routeCallback(const route_planning_msgs::msg::Route::UniquePtr msg);

  void publishTimerCallback();

  /**
   * @brief Determines the current planner state from input freshness and route availability.
   *
   * @param[in] stamp Current planning timestamp.
   * @return PlannerState The state to be handled for this cycle.
   */
  PlannerState determinePlannerState(const rclcpp::Time& stamp);

  /**
   * @brief Checks whether an input message is older than the configured timeout.
   *
   * @param[in] header Header of the input message.
   * @param[in] timeout Timeout in seconds. A value of `-1.0` disables the timeout.
   * @param[in] stamp Current planning timestamp.
   * @return true if the message is outdated.
   * @return false if the message is still valid.
   */
  bool isMessageOutdated(const std_msgs::msg::Header& header, double timeout, const rclcpp::Time& stamp) const;

  /**
   * @brief Creates a trajectory for the already determined planner state.
   *
   * @param[in] state Planner state to be executed.
   * @param[in] stamp Current planning timestamp propagated from the timer callback.
   * @return trajectory_planning_msgs::msg::Trajectory The generated output trajectory.
   */
  trajectory_planning_msgs::msg::Trajectory createTrajectory(PlannerState state, const rclcpp::Time& stamp);

  /**
   * @brief Creates a standstill trajectory for the current planning cycle.
   *
   * @param[in] target_header Output header for the generated trajectory.
   * @return trajectory_planning_msgs::msg::Trajectory Standstill trajectory message.
   */
  trajectory_planning_msgs::msg::Trajectory buildStandstillTrajectory(const std_msgs::msg::Header& target_header);

  /**
   * @brief Builds a trajectory message from a simple path.
   *
   * This also trims points behind the ego vehicle and falls back to standstill if
   * no usable forward path remains.
   *
   * @param[in] path Path to be converted.
   * @return trajectory_planning_msgs::msg::Trajectory Generated trajectory message.
   */
  trajectory_planning_msgs::msg::Trajectory buildTrajectoryFromSimplePath(const SimplePath& path);
  void clearObjectInteractionMarkers(const std_msgs::msg::Header& target_header);
  void publishObjectInteractionMarkers(const std_msgs::msg::Header& target_header,
                                       const std::optional<ConflictSample>& conflict);

  /**
   * @brief Builds the initial safe-stop path for the current cycle.
   *
   * @param[in] target_header Output header for the generated safe-stop path.
   * @return SimplePath Safe-stop path in trajectory frame.
   */
  SimplePath buildSafeStopPath(const std_msgs::msg::Header& target_header);

  /**
   * @brief Builds the complete route-following plan including stop and turn information.
   *
   * @param[in] target_header Output header for the generated route plan.
   * @return FollowRoutePlan Planned route path including stop and turn information.
   */
  FollowRoutePlan buildRoutePlan(const std_msgs::msg::Header& target_header);

  /**
   * @brief Applies trajectory-based object conflict constraints to a follow-route plan.
   *
   * The object list is transformed into trajectory frame and checked against the
   * already planned ego trajectory. If conflicts are detected, the path is
   * resampled repeatedly with a lower speed cap until it is conflict-free or
   * the speed cap reaches zero.
   *
   * @param target_header Current planning header propagated from the timer callback.
   * @param base_path_points Route path before time-based resampling.
   * @param route_plan Mutable route plan to be constrained by dynamic objects.
   */
  void applyObjectConstraints(const std_msgs::msg::Header& target_header, const std::vector<SimplePathPoint>& base_path_points,
                              FollowRoutePlan& route_plan);

  /**
   * @brief Reduces the perceived object list (in trajectory frame) to timed bounding-box trajectories.
   *
   * Static objects (without usable prediction) become a single-sample trajectory; otherwise the
   * predictions above the probability threshold (or the most likely one) are sampled.
   *
   * @param[in] tf_object_list Object list transformed into trajectory frame.
   * @param[in] stamp Current planning timestamp used to compute relative sample times.
   * @return Timed bounding-box trajectories for all relevant objects.
   */
  std::vector<ObjectTrajectory> buildObjectTrajectories(const perception_msgs::msg::ObjectList& tf_object_list,
                                                        const rclcpp::Time& stamp) const;

  /**
   * @brief Returns the first conflict between the (time-sampled) ego path and any object trajectory.
   *
   * @param[in] ego_path Time-equidistant ego path candidate.
   * @param[in] object_trajectories Object trajectories to check against.
   * @return The first detected conflict, or std::nullopt if the ego path is conflict-free.
   */
  std::optional<ConflictSample> firstConflict(const std::vector<SimplePathPoint>& ego_path,
                                              const std::vector<ObjectTrajectory>& object_trajectories) const;

  /**
   * @brief Resets the remembered object speed cap / hysteresis state and clears interaction markers.
   *
   * @param[in] target_header Header used for the cleared marker messages.
   */
  void resetObjectState(const std_msgs::msg::Header& target_header);

  /**
   * @brief Appends route-derived path points and stop metadata for the follow-route case.
   *
   * @param[in] tf_route Route transformed into trajectory frame.
   * @param[in,out] route_plan Mutable route planning result to extend.
   * @param[out] lane_change_indices_map Output lane-change windows to be merged later.
   */
  void appendRoutePoints(const route_planning_msgs::msg::Route& tf_route, FollowRoutePlan& route_plan,
                         std::map<uint64_t, uint64_t>& lane_change_indices_map);

  /**
   * @brief Detects and stores the start/end window of a lane change.
   *
   * @param[in] tf_route Route transformed into trajectory frame.
   * @param[in] route_element_idx Index of the current route element.
   * @param[out] lane_change_indices_map Output map of lane-change route indices.
   * @param[in,out] suggested_turn_signal Turn signal selected for the current route plan.
   * @return true if the lane change could be registered.
   * @return false if the route data is insufficient and processing should stop.
   */
  bool tryRegisterLaneChange(const route_planning_msgs::msg::Route& tf_route, size_t route_element_idx,
                             std::map<uint64_t, uint64_t>& lane_change_indices_map, uint8_t& suggested_turn_signal);

  /**
   * @brief Updates stop-at-end and stop-line offset state for traffic-light regulatory elements.
   *
   * @param[in] tf_route Route transformed into trajectory frame.
   * @param[in] route_element_idx Index of the current route element.
   * @param[in] suggested_lane Suggested lane element of the current route element.
   * @param[in] simple_path_point Current path point candidate.
   * @param[in] t_total Accumulated travel time along the partial path.
   * @param[in,out] stop_at_end Whether the path should stop at its current end.
   * @param[in,out] offset_to_stop_line Effective offset used for braking towards the stop line.
   */
  void updateForTrafficLights(const route_planning_msgs::msg::Route& tf_route, size_t route_element_idx,
                              const route_planning_msgs::msg::LaneElement& suggested_lane,
                              const SimplePathPoint& simple_path_point, double t_total,
                              bool& stop_at_end, double& offset_to_stop_line);

  /**
   * @brief Merges interpolated lane-change segments into the base route path.
   *
   * The base route points are the already filtered route-following path, while
   * `tf_route` is still needed to reconstruct the lane geometry for the
   * interpolated lane-change segments.
   *
   * @param[in] tf_route Route transformed into trajectory frame and used to derive lane-change geometry.
   * @param[in] route_points Base route points before lane-change insertion.
   * @param[in] lane_change_indices_map Lane-change windows gathered during route processing.
   * @return std::vector<SimplePathPoint> Path points with lane changes inserted.
   */
  std::vector<SimplePathPoint> mergeLaneChangeSegments(const route_planning_msgs::msg::Route& tf_route,
                                                       const std::vector<SimplePathPoint>& route_points,
                                                       const std::map<uint64_t, uint64_t>& lane_change_indices_map);

  /**
   * @brief Requests the appropriate indicator state for the current route plan.
   *
   * @param[in] suggested_turn_signal Suggested signal derived from route semantics.
   */
  void applyIndicatorRequest(uint8_t suggested_turn_signal);

  /**
   * @brief Removes path points that lie behind the ego vehicle in trajectory frame.
   *
   * @param[in,out] path Path to be trimmed in-place.
   */
  void trimPathBehindEgo(SimplePath& path);

  std::vector<SimplePathPoint> resamplePath(const std::vector<SimplePathPoint>& path, bool stop_at_end, double offset_to_stop_line = 0.0,
                                            const double* speed_cap = nullptr);
  std::vector<SimplePathPoint> generateLaneChangePath(const int start_idx, const int turn_idx,
                                                      const route_planning_msgs::msg::Route& route);
  void recalculateS(std::vector<SimplePathPoint>& path);
  SimplePath calculateSafeStopAlongEgoHeading(const perception_msgs::msg::EgoData& ego_data, const double safe_stop_distance, const std_msgs::msg::Header& target_header);
  SimplePath calculateSafeStopAlongRoute(const SimplePath& path, const double safe_stop_distance);
  SimplePath transformPath(const SimplePath& path, const std_msgs::msg::Header& target_header);

  /**
   * @brief Function called by diagnostic updater to populate diagnostics status
   */
  void health(diagnostic_updater::DiagnosticStatusWrapper& stat);

  /**
   * @brief Sets the health information
   */
  void setHealth(const unsigned char status, const std::string& msg,
                 const std::map<std::string, std::string>& key_value_pairs = {});

  /**
   * @brief Converts a PlannerState enum to a string representation
   *
   * @param state
   * @return std::string
   */
  std::string plannerStatetoString(const PlannerState& state) const;

  /**
   * @brief Converts a turn signal value to a string representation
   *
   * @param turn_signal
   * @return std::string
   */
  std::string turnSignalToString(const uint8_t& turn_signal) const;

  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  rclcpp::Subscription<perception_msgs::msg::EgoData>::SharedPtr sub_egoData_;
  rclcpp::Subscription<perception_msgs::msg::ObjectList>::SharedPtr sub_object_list_;
  rclcpp::Subscription<route_planning_msgs::msg::Route>::SharedPtr sub_route_;

  rclcpp::Publisher<trajectory_planning_msgs::msg::Trajectory>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr object_interaction_marker_pub_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr left_turn_indicator_service_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr right_turn_indicator_service_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr hazard_lights_service_client_;

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
  double object_timeout_ = 1.0;
  double trajectory_horizon_ = 10.0;
  int n_states_ = 51;
  uint8_t interpolation_type_ = InterpolationType::SPLINE;
  double v_ref_ = 13.89;
  double a_decel_ = -0.5;
  double a_max_decel_ = -1.0;
  bool trigger_turn_signals_ = true;
  bool consider_traffic_lights_ = true;
  double offset_to_stop_line_ = 0.0;
  double ignore_stop_line_threshold_ = 0.5;
  bool consider_future_states_ = false;
  bool consider_objects_ = true;
  double min_prediction_prob_ = 0.0;
  double object_longitudinal_safety_distance_ = 1.5;
  double object_lateral_safety_distance_ = 0.0;
  double object_interaction_time_window_ = 0.5;
  double object_velocity_reduction_step_ = 0.1;
  double object_velocity_release_step_ = 0.5;
  double object_standstill_speed_threshold_ = 0.2;
  int object_velocity_release_hysteresis_cycles_ = 3;
  bool publish_object_interaction_markers_ = true;

  double lane_change_distance_factor_ = 6.0;
  double lane_change_min_distance_factor_ = 2.0;

  perception_msgs::msg::EgoData ego_data_;
  perception_msgs::msg::ObjectList object_list_;
  route_planning_msgs::msg::Route route_;

  bool ego_data_init_ = false;
  bool object_list_init_ = false;
  bool route_init_ = false;
  std::optional<double> safe_stop_distance_;
  std::optional<double> last_object_speed_cap_;
  SimplePath latest_path_;
  int object_conflict_free_cycles_ = 0;
  double dt_;

  /**
   * @brief Diagnostic updater
   */
  diagnostic_updater::Updater diagnostic_updater_{this};

  /**
   * @brief Diagnostic status indicating node health
   */
  struct DiagnosticStatus {
    unsigned char status = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    std::string message = "";
    std::map<std::string, std::string> key_value_pairs = {};
  } health_;

  std::unique_ptr<diagnostic_updater::TopicDiagnostic> ego_data_topic_diagnostic_;
  TopicDiagnosticConfig ego_data_topic_diagnostic_config_{45.45, 55.55, 0.0, 0.002};

  std::unique_ptr<diagnostic_updater::TopicDiagnostic> object_list_topic_diagnostic_;
  TopicDiagnosticConfig object_list_topic_diagnostic_config_{9.09, 11.11, 0.0, 0.01};

  std::unique_ptr<diagnostic_updater::TopicDiagnostic> route_topic_diagnostic_;
  TopicDiagnosticConfig route_topic_diagnostic_config_{18.18, 22.22, 0.0, 0.005};

  std::unique_ptr<diagnostic_updater::DiagnosedPublisher<trajectory_planning_msgs::msg::Trajectory>>
      diagnosed_publisher_;
  TopicDiagnosticConfig diagnosed_publisher_config_{9.09, 11.11, 0.0, 0.01};
};

}  // namespace simple_planner
