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

void SimplePlannerNode::clearObjectInteractionMarkers(const std_msgs::msg::Header& target_header) {
  if (!object_interaction_marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  const std::array<std::string, 4> namespaces = {
      "object_interaction_ego_safety_box",
      "object_interaction_ego_box",
      "object_interaction_object_box",
      "object_interaction_text"};
  for (size_t idx = 0; idx < namespaces.size(); ++idx) {
    visualization_msgs::msg::Marker marker;
    marker.header = target_header;
    marker.ns = namespaces[idx];
    marker.id = static_cast<int>(idx);
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.push_back(marker);
  }
  object_interaction_marker_pub_->publish(marker_array);
}

void SimplePlannerNode::publishObjectInteractionMarkers(const std_msgs::msg::Header& target_header,
                                                        const std::optional<ObjectConflictDebugSample>& debug_sample) {
  if (!object_interaction_marker_pub_) {
    return;
  }

  if (!publish_object_interaction_markers_) {
    clearObjectInteractionMarkers(target_header);
    return;
  }
  if (!debug_sample.has_value()) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  const auto& sample = *debug_sample;
  auto make_box_marker = [&](const std::string& ns, int marker_id, const geometry_msgs::msg::Pose& pose,
                             double length, double width, double z, float r, float g, float b, float a) {
    visualization_msgs::msg::Marker marker;
    marker.header = target_header;
    marker.ns = ns;
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose;
    marker.pose.position.z = z;
    marker.scale.x = length;
    marker.scale.y = width;
    marker.scale.z = 0.08;
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 500000000;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker_array.markers.push_back(marker);
  };

  make_box_marker("object_interaction_ego_safety_box", 0, sample.ego_pose,
                  sample.ego_safety_length, sample.ego_safety_width,
                  0.12, 1.0f, 0.55f, 0.0f, 0.28f);
  make_box_marker("object_interaction_ego_box", 1, sample.ego_pose,
                  sample.ego_length, sample.ego_width,
                  0.18, 1.0f, 0.0f, 0.0f, 0.55f);
  make_box_marker("object_interaction_object_box", 2, sample.object_pose,
                  sample.object_length, sample.object_width,
                  0.24, 0.0f, 0.45f, 1.0f, 0.55f);

  visualization_msgs::msg::Marker text_marker;
  text_marker.header = target_header;
  text_marker.ns = "object_interaction_text";
  text_marker.id = 3;
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::msg::Marker::ADD;
  text_marker.pose = sample.ego_pose;
  text_marker.pose.position.z = 0.65;
  text_marker.pose.orientation.x = 0.0;
  text_marker.pose.orientation.y = 0.0;
  text_marker.pose.orientation.z = 0.0;
  text_marker.pose.orientation.w = 1.0;
  text_marker.scale.z = 0.3;
  text_marker.lifetime.sec = 0;
  text_marker.lifetime.nanosec = 500000000;
  text_marker.color.r = 1.0f;
  text_marker.color.g = 1.0f;
  text_marker.color.b = 1.0f;
  text_marker.color.a = 0.9f;
  std::ostringstream text_stream;
  text_stream << "object=" << sample.object_id << " t=" << sample.t
              << " v_cap=" << sample.speed_cap << " it=" << sample.iteration;
  text_marker.text = text_stream.str();
  marker_array.markers.push_back(text_marker);

  object_interaction_marker_pub_->publish(marker_array);
}

} // namespace simple_planner
