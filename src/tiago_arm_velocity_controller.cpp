#include "tiago_arm_velocity_controller/tiago_arm_velocity_controller.hpp"
#include <algorithm>
#include <cmath>
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
  config.names.reserve(joint_names_.size() * 2);
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TiagoArmVelocityPositionController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::vector<double>>("p_gains", std::vector<double>());
    auto_declare<std::vector<double>>("i_gains", std::vector<double>());
    auto_declare<std::vector<double>>("d_gains", std::vector<double>());
    auto_declare<std::vector<double>>("feedforward_gains", std::vector<double>());
    auto_declare<std::vector<double>>("max_accelerations", std::vector<double>());
    auto_declare<double>("cmd_timeout_seconds", 0.2);
    auto_declare<double>("velocity_filter_alpha", 0.95);
    auto_declare<std::string>("robot_description", std::string(""));
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
    RCLCPP_ERROR(node->get_logger(), "'joints' parameter vector cannot be empty.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Load and validate gain sizes
  p_gains_ = node->get_parameter("p_gains").as_double_array();
  i_gains_ = node->get_parameter("i_gains").as_double_array();
  d_gains_ = node->get_parameter("d_gains").as_double_array();
  ff_gains_ = node->get_parameter("feedforward_gains").as_double_array();
  max_accelerations_ = node->get_parameter("max_accelerations").as_double_array();

  if (p_gains_.size() != n_joints_ || i_gains_.size() != n_joints_ ||
      d_gains_.size() != n_joints_ || ff_gains_.size() != n_joints_)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Gains vector sizes (P: %zu, I: %zu, D: %zu, FF: %zu) do not match joint count (%zu)",
      p_gains_.size(), i_gains_.size(), d_gains_.size(), ff_gains_.size(), n_joints_);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // If max_accelerations is omitted, default to 2.0 rad/s^2 per joint
  if (max_accelerations_.empty()) {
    max_accelerations_.assign(n_joints_, 2.0);
  } else if (max_accelerations_.size() != n_joints_) {
    RCLCPP_ERROR(node->get_logger(), "'max_accelerations' size must match number of joints.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  timeout_seconds_ = node->get_parameter("cmd_timeout_seconds").as_double();
  filter_alpha_ = static_cast<float>(node->get_parameter("velocity_filter_alpha").as_double());

  // Parse URDF joint limits
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
          joint_limits_[i].lower = joint->limits->lower;
          joint_limits_[i].upper = joint->limits->upper;
        }
      }
    }
  }

  // Initialize control vectors
  js_.assign(n_joints_, 0.0);
  old_integrator_.assign(n_joints_, 0.0);
  old_error_.assign(n_joints_, 0.0);
  filtered_vel_.assign(n_joints_, 0.0);
  ramped_vel_cmd_.assign(n_joints_, 0.0);

  CommandData init_cmd;
  init_cmd.velocities.assign(n_joints_, 0.0);
  init_cmd.timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  commands_buffer_.writeFromNonRT(init_cmd);

  // Command Subscription
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

  // Real-Time Publisher for filtered velocities / debug state
  auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/controller_state", rclcpp::SystemDefaultsQoS());
  pub_controller_state_ =
    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(publisher);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TiagoArmVelocityPositionController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  pos_state_indices_.clear();
  vel_state_indices_.clear();

  // Explicit, safe interface resolution
  for (const auto & joint_name : joint_names_) {
    bool found_pos = false;
    bool found_vel = false;

    for (size_t i = 0; i < state_interfaces_.size(); ++i) {
      const auto & iface = state_interfaces_[i];
      if (iface.get_prefix_name() == joint_name) {
        if (iface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
          pos_state_indices_.push_back(i);
          found_pos = true;
        } else if (iface.get_interface_name() == hardware_interface::HW_IF_VELOCITY) {
          vel_state_indices_.push_back(i);
          found_vel = true;
        }
      }
    }

    if (!found_pos || !found_vel) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to resolve position or velocity state interface for joint: %s",
        joint_name.c_str());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }

  // Latch current physical state for bumpless start
  for (size_t i = 0; i < n_joints_; ++i) {
    double current_pos = state_interfaces_[pos_state_indices_[i]].get_value();
    double current_vel = state_interfaces_[vel_state_indices_[i]].get_value();

    js_[i] = current_pos;
    old_integrator_[i] = 0.0;
    old_error_[i] = 0.0;
    filtered_vel_[i] = current_vel;
    ramped_vel_cmd_[i] = 0.0;

    command_interfaces_[i].set_value(current_pos);
  }

  // Clear incoming buffer
  CommandData reset_cmd;
  reset_cmd.velocities.assign(n_joints_, 0.0);
  reset_cmd.timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  commands_buffer_.writeFromNonRT(reset_cmd);

  update_counter_ = 0;
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
  const double dt = period.seconds();
  if (dt <= 1e-7) {
    return controller_interface::return_type::OK;
  }

  const auto & cmd_data = *commands_buffer_.readFromRT();

  // Watchdog verification
  const bool is_timed_out =
    (cmd_data.timestamp.nanoseconds() == 0) ||
    ((time - cmd_data.timestamp).seconds() > timeout_seconds_);

  for (size_t i = 0; i < n_joints_; ++i) {
    const double target_vel = is_timed_out ? 0.0 : cmd_data.velocities[i];

    // Anti-jerk acceleration limiter (ramping)
    const double max_delta_v = max_accelerations_[i] * dt;
    const double vel_diff = target_vel - ramped_vel_cmd_[i];
    if (std::abs(vel_diff) > max_delta_v) {
      ramped_vel_cmd_[i] += std::copysign(max_delta_v, vel_diff);
    } else {
      ramped_vel_cmd_[i] = target_vel;
    }

    const double current_pos = state_interfaces_[pos_state_indices_[i]].get_value();
    const double current_vel = state_interfaces_[vel_state_indices_[i]].get_value();

    // Exponential moving average filter on encoder velocity
    filtered_vel_[i] = (1.0f - filter_alpha_) * filtered_vel_[i] + filter_alpha_ * current_vel;

    if (std::abs(ramped_vel_cmd_[i]) < 1e-6) {
      // Hold state when target is zero
      js_[i] = current_pos;
      old_integrator_[i] = 0.0;
      old_error_[i] = 0.0;
    } else {
      const double error = ramped_vel_cmd_[i] - filtered_vel_[i];
      const double new_integrator = old_integrator_[i] + error * dt;

      // Original HSR Feedforward + PID control law
      double next_pos = current_pos + (ramped_vel_cmd_[i] * dt * ff_gains_[i])
                        + (error * p_gains_[i])
                        + (new_integrator * i_gains_[i])
                        + (d_gains_[i] / dt * (error - old_error_[i]));

      // Clamp output by URDF limits and manage anti-windup
      if (joint_limits_[i].has_limits) {
        if (next_pos > joint_limits_[i].upper) {
          next_pos = joint_limits_[i].upper;
          if (error < 0.0) {
            old_integrator_[i] = new_integrator;
          }
        } else if (next_pos < joint_limits_[i].lower) {
          next_pos = joint_limits_[i].lower;
          if (error > 0.0) {
            old_integrator_[i] = new_integrator;
          }
        } else {
          old_integrator_[i] = new_integrator;
        }
      } else {
        old_integrator_[i] = new_integrator;
      }

      js_[i] = next_pos;
      old_error_[i] = error;
    }

    command_interfaces_[i].set_value(js_[i]);
  }

  // Publish controller state at 100 Hz (every 10 ticks at 1 kHz)
  if (update_counter_ % 10 == 0 && pub_controller_state_ && pub_controller_state_->trylock()) {
    pub_controller_state_->msg_.data = filtered_vel_;
    pub_controller_state_->unlockAndPublish();
  }
  update_counter_++;

  return controller_interface::return_type::OK;
}

}  // namespace tiago_arm_controllers

PLUGINLIB_EXPORT_CLASS(
  tiago_arm_controllers::TiagoArmVelocityPositionController,
  controller_interface::ControllerInterface)