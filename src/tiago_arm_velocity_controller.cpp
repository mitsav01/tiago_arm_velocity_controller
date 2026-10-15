#include "tiago_arm_velocity_controller/tiago_arm_velocity_controller.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace tiago_arm_controllers
{

controller_interface::InterfaceConfiguration
TiagoArmVelocityPositionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(joint_names_.size());
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

controller_interface::InterfaceConfiguration
TiagoArmVelocityPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(joint_names_.size());
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TiagoArmVelocityPositionController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::string>("robot_description", std::string(""));
    auto_declare<double>("cmd_timeout_seconds", 0.2);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TiagoArmVelocityPositionController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();

  joint_names_ = node->get_parameter("joints").as_string_array();
  n_joints_ = joint_names_.size();

  if (n_joints_ == 0) {
    RCLCPP_ERROR(node->get_logger(), "'joints' parameter cannot be empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  timeout_seconds_ = node->get_parameter("cmd_timeout_seconds").as_double();

  // Load URDF joint limits if available
  joint_limits_.assign(n_joints_, JointLimit{false, 0.0, 0.0});
  std::string urdf_xml = "";
  if (node->has_parameter("robot_description")) {
    urdf_xml = node->get_parameter("robot_description").as_string();
  }

  if (!urdf_xml.empty()) {
    urdf::Model model;
    if (model.initString(urdf_xml)) {
      for (size_t i = 0; i < n_joints_; ++i) {
        auto joint = model.getJoint(joint_names_[i]);
        if (joint && joint->limits) {
          joint_limits_[i].has_limits = true;
          joint_limits_[i].min_position = joint->limits->lower;
          joint_limits_[i].max_position = joint->limits->upper;
        }
      }
    }
  }

  CommandData init_cmd;
  init_cmd.velocities.assign(n_joints_, 0.0);
  init_cmd.timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  commands_buffer_.writeFromNonRT(init_cmd);

  target_pos_.assign(n_joints_, 0.0);

  sub_command_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      if (msg->data.size() == n_joints_) {
        CommandData cmd;
        cmd.velocities = msg->data;
        cmd.timestamp = get_node()->now();
        commands_buffer_.writeFromNonRT(cmd);
      }
    });

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TiagoArmVelocityPositionController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  pos_state_indices_.clear();

  for (const auto & joint_name : joint_names_) {
    bool found_pos = false;
    for (size_t i = 0; i < state_interfaces_.size(); ++i) {
      const auto & iface = state_interfaces_[i];
      if (iface.get_prefix_name() == joint_name &&
          iface.get_interface_name() == hardware_interface::HW_IF_POSITION)
      {
        pos_state_indices_.push_back(i);
        found_pos = true;
        break;
      }
    }

    if (!found_pos) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing position state interface for %s", joint_name.c_str());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }

  // Latch current physical joint positions
  for (size_t i = 0; i < n_joints_; ++i) {
    double current_p = state_interfaces_[pos_state_indices_[i]].get_value();
    target_pos_[i] = current_p;
    command_interfaces_[i].set_value(current_p);
  }

  // Invalidate any buffered commands prior to activation
  CommandData reset_cmd;
  reset_cmd.velocities.assign(n_joints_, 0.0);
  reset_cmd.timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  commands_buffer_.writeFromNonRT(reset_cmd);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TiagoArmVelocityPositionController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
TiagoArmVelocityPositionController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const auto & cmd_data = *commands_buffer_.readFromRT();
  const double dt = period.seconds();

  if (dt <= 0.0) {
    return controller_interface::return_type::OK;
  }

  // Watchdog: verify message freshness against controller cycle time
  const bool is_timed_out =
    (cmd_data.timestamp.nanoseconds() == 0) ||
    ((time - cmd_data.timestamp).seconds() > timeout_seconds_);

  for (size_t i = 0; i < n_joints_; ++i) {
    const double vel_cmd = is_timed_out ? 0.0 : cmd_data.velocities[i];

    if (std::abs(vel_cmd) > 1e-6) {
      target_pos_[i] += vel_cmd * dt;

      if (joint_limits_[i].has_limits) {
        if (target_pos_[i] > joint_limits_[i].max_position) {
          target_pos_[i] = joint_limits_[i].max_position;
        } else if (target_pos_[i] < joint_limits_[i].min_position) {
          target_pos_[i] = joint_limits_[i].min_position;
        }
      }
    }

    command_interfaces_[i].set_value(target_pos_[i]);
  }

  return controller_interface::return_type::OK;
}

}  // namespace tiago_arm_controllers

PLUGINLIB_EXPORT_CLASS(
  tiago_arm_controllers::TiagoArmVelocityPositionController,
  controller_interface::ControllerInterface)