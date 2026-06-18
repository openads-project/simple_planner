namespace simple_planner {

template <typename T>
void SimplePlannerNode::declareAndLoadParameter(const std::string& name,
                                                T& param,
                                                const std::string& description,
                                                const bool add_to_auto_reconfigurable_params,
                                                const bool is_required,
                                                const bool read_only,
                                                const std::optional<double>& from_value,
                                                const std::optional<double>& to_value,
                                                const std::optional<double>& step_value,
                                                const std::string& additional_constraints) {

  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = description;
  param_desc.additional_constraints = additional_constraints;
  param_desc.read_only = read_only;

  auto type = rclcpp::ParameterValue(param).get_type();

  if (from_value.has_value() && to_value.has_value()) {
    if constexpr(std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1);
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value())).set__step(step);
      param_desc.integer_range = {range};
    } else if constexpr(std::is_floating_point_v<T>) {
      rcl_interfaces::msg::FloatingPointRange range;
      T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1.0);
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value())).set__step(step);
      param_desc.floating_point_range = {range};
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter type of parameter '%s' does not support specifying a range", name.c_str());
    }
  }

  this->declare_parameter(name, type, param_desc);

  try {
    param = this->get_parameter(name).get_value<T>();
    std::stringstream ss;
    ss << "Loaded parameter '" << name << "': ";
    if constexpr(is_vector_v<T>) {
      ss << "[";
      for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "");
      ss << "]";
    } else {
      ss << param;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    if (is_required) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Missing required parameter '" << name << "', exiting");
      exit(EXIT_FAILURE);
    } else {
      std::stringstream ss;
      ss << "Missing parameter '" << name << "', using default value: ";
      if constexpr(is_vector_v<T>) {
        ss << "[";
        for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "");
        ss << "]";
      } else {
        ss << param;
      }
      RCLCPP_WARN_STREAM(this->get_logger(), ss.str());
      this->set_parameters({rclcpp::Parameter(name, rclcpp::ParameterValue(param))});
    }
  }

  if (add_to_auto_reconfigurable_params) {
    std::function<void(const rclcpp::Parameter&)> setter = [&param](const rclcpp::Parameter& p) {
      param = p.get_value<T>();
    };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}


rcl_interfaces::msg::SetParametersResult SimplePlannerNode::parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;

  for (const auto& param : parameters) {

    // check for specific parameter constraints
    if (param.get_name() == "a_decel") {
      if (param.as_double() >= 0.0) {
        result.successful = false;
        result.reason = "a_decel (" + std::to_string(param.as_double()) + ") must be < 0.0";
        RCLCPP_WARN(this->get_logger(), "Rejected parameter change for 'a_decel': %s", result.reason.c_str());
        break;
      } else if (a_max_decel_ > param.as_double()) {
        result.successful = false;
        result.reason = "a_max_decel (" + std::to_string(a_max_decel_) + ") must be <= a_decel (" + std::to_string(param.as_double()) + ")";
        RCLCPP_WARN(this->get_logger(), "Rejected parameter change for 'a_decel': %s", result.reason.c_str());
        break;
      }
    } else if (param.get_name() == "a_max_decel") {
      if (param.as_double() >= 0.0) {
        result.successful = false;
        result.reason = "a_max_decel (" + std::to_string(param.as_double()) + ") must be < 0.0";
        RCLCPP_WARN(this->get_logger(), "Rejected parameter change for 'a_max_decel': %s", result.reason.c_str());
        break;
      } else if (param.as_double() > a_decel_) {
        result.successful = false;
        result.reason = "a_max_decel (" + std::to_string(param.as_double()) + ") must be <= a_decel (" + std::to_string(a_decel_) + ")";
        RCLCPP_WARN(this->get_logger(), "Rejected parameter change for 'a_max_decel': %s", result.reason.c_str());
        break;
      }
    }

    // apply parameter change
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
        RCLCPP_INFO(this->get_logger(), "Reconfigured parameter '%s'", param.get_name().c_str());
        result.successful = true;
        break;
      }
    }
  }

  return result;
}

SimplePath SimplePlannerNode::transformPath(const SimplePath& path, const std_msgs::msg::Header& target_header) {
  SimplePath transformed_path;
  transformed_path.header = target_header;

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf2_buffer_->lookupTransform(target_header.frame_id, target_header.stamp, path.header.frame_id, path.header.stamp,
                                      fixed_over_time_frame_id_, rclcpp::Duration::from_seconds(1.0));
    for (const auto& point : path.points) {
      geometry_msgs::msg::PointStamped point_msg, transformed_point_msg;
      point_msg.header = path.header;
      point_msg.point.x = point.position.x();
      point_msg.point.y = point.position.y();
      point_msg.point.z = 0.0;
      tf2::doTransform(point_msg, transformed_point_msg, tf);
      SimplePathPoint transformed_point = point;
      transformed_point.position = Eigen::Vector2d(transformed_point_msg.point.x, transformed_point_msg.point.y);
      transformed_path.points.push_back(transformed_point);
    }
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform path: %s. Reusing old path.", ex.what());
    return path;
  }

  return transformed_path;
}


void SimplePlannerNode::health(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  stat.summary(health_.status, health_.message);
  for (const auto& [key, value] : health_.key_value_pairs) {
    stat.add(key, value);
  }
}

void SimplePlannerNode::setHealth(const unsigned char status, const std::string& msg, const std::map<std::string, std::string>& key_value_pairs) {
  health_.status = status;
  health_.message = msg;
  health_.key_value_pairs = key_value_pairs;
}

std::string SimplePlannerNode::plannerStateToString(const SimplePlannerNode::PlannerState& state) const {
  switch (state) {
    case PlannerState::NoPublish:
      return "NoPublish";
    case PlannerState::Standstill:
      return "Standstill";
    case PlannerState::SafeStop:
      return "SafeStop";
    case PlannerState::FollowRoute:
      return "FollowRoute";
    default:
      return "Unknown";
  }
}

std::string SimplePlannerNode::turnSignalToString(const uint8_t& turn_signal) const {
  switch (turn_signal) {
    case route_planning_msgs::msg::LaneElement::SUGGESTED_TURN_SIGNAL_NONE:
      return "None";
    case route_planning_msgs::msg::LaneElement::SUGGESTED_TURN_SIGNAL_LEFT:
      return "Left";
    case route_planning_msgs::msg::LaneElement::SUGGESTED_TURN_SIGNAL_RIGHT:
      return "Right";
    case route_planning_msgs::msg::LaneElement::SUGGESTED_TURN_SIGNAL_HAZARD:
      return "Hazard";
    default:
      return "Unknown";
  }
}

} // namespace simple_planner
