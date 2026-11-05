#include <vector>
#include <string>
#include <memory>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <urdf/model.h>

namespace tiago_velocity_controller_ns
{

struct CommandData
{
  std::vector<double> velocities;
  std::chrono::steady_clock::time_point timestamp;
  bool received{false};
};

class TiagoVelocityController : public controller_interface::ControllerInterface
{
public:
  TiagoVelocityController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(joint_names_.size());
    for (const auto &joint_name : joint_names_) {
      config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    }
    return config;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(joint_names_.size() * 2);
    for (const auto &joint_name : joint_names_) {
      config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
      config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return config;
  }

  controller_interface::CallbackReturn on_init() override
  {
    auto node = get_node();

    if (!node->has_parameter("joints")) {
      node->declare_parameter("joints", std::vector<std::string>());
    }
    if (!node->has_parameter("p_gains")) {
      node->declare_parameter("p_gains", std::vector<double>());
    }
    if (!node->has_parameter("i_gains")) {
      node->declare_parameter("i_gains", std::vector<double>());
    }
    if (!node->has_parameter("d_gains")) {
      node->declare_parameter("d_gains", std::vector<double>());
    }
    if (!node->has_parameter("feedforward_gains")) {
      node->declare_parameter("feedforward_gains", std::vector<double>());
    }
    if (!node->has_parameter("cmd_timeout_seconds")) {
      node->declare_parameter("cmd_timeout_seconds", 0.2);
    }
    if (!node->has_parameter("max_velocities")) {
      node->declare_parameter("max_velocities", std::vector<double>());
    }
    if (!node->has_parameter("max_accelerations")) {
      node->declare_parameter("max_accelerations", std::vector<double>());
    }
    if (!node->has_parameter("max_position_error")) {
      node->declare_parameter("max_position_error", std::vector<double>());
    }
    if (!node->has_parameter("velocity_filter_alpha")) {
      node->declare_parameter("velocity_filter_alpha", 0.95);
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    auto node = get_node();

    joint_names_ = node->get_parameter("joints").as_string_array();
    n_joints_ = joint_names_.size();

    if (n_joints_ == 0) {
      RCLCPP_ERROR(node->get_logger(), "Parameter 'joints' must not be empty.");
      return controller_interface::CallbackReturn::ERROR;
    }

    // Verify unique joint names
    std::unordered_set<std::string> unique_joints(joint_names_.begin(), joint_names_.end());
    if (unique_joints.size() != n_joints_) {
      RCLCPP_ERROR(node->get_logger(), "Duplicate joint names detected in 'joints' parameter.");
      return controller_interface::CallbackReturn::ERROR;
    }

    p_gains_ = node->get_parameter("p_gains").as_double_array();
    i_gains_ = node->get_parameter("i_gains").as_double_array();
    d_gains_ = node->get_parameter("d_gains").as_double_array();
    ff_gains_ = node->get_parameter("feedforward_gains").as_double_array();
    max_velocities_ = node->get_parameter("max_velocities").as_double_array();
    max_accelerations_ = node->get_parameter("max_accelerations").as_double_array();
    max_position_error_ = node->get_parameter("max_position_error").as_double_array();
    cmd_timeout_sec_ = node->get_parameter("cmd_timeout_seconds").as_double();
    filter_alpha_ = node->get_parameter("velocity_filter_alpha").as_double();

    // Dimensional check
    if (p_gains_.size() != n_joints_ || i_gains_.size() != n_joints_ ||
        d_gains_.size() != n_joints_ || ff_gains_.size() != n_joints_ ||
        max_velocities_.size() != n_joints_ || max_accelerations_.size() != n_joints_ ||
        max_position_error_.size() != n_joints_) {
      RCLCPP_ERROR(node->get_logger(),
                   "Array dimension mismatch! Expected %zu elements for all per-joint parameters.", n_joints_);
      return controller_interface::CallbackReturn::ERROR;
    }

    // Scalar parameter validations
    if (!std::isfinite(cmd_timeout_sec_) || cmd_timeout_sec_ <= 0.0) {
      RCLCPP_ERROR(node->get_logger(), "cmd_timeout_seconds must be finite and > 0.0. Given: %f", cmd_timeout_sec_);
      return controller_interface::CallbackReturn::ERROR;
    }
    if (!std::isfinite(filter_alpha_) || filter_alpha_ <= 0.0 || filter_alpha_ > 1.0) {
      RCLCPP_ERROR(node->get_logger(), "velocity_filter_alpha must be in (0.0, 1.0]. Given: %f", filter_alpha_);
      return controller_interface::CallbackReturn::ERROR;
    }

    // Per-joint bounds validation
    for (size_t i = 0; i < n_joints_; ++i) {
      if (!std::isfinite(max_velocities_[i]) || max_velocities_[i] <= 0.0) {
        RCLCPP_ERROR(node->get_logger(), "Joint %s: max_velocities must be finite and > 0.0", joint_names_[i].c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      if (!std::isfinite(max_accelerations_[i]) || max_accelerations_[i] <= 0.0) {
        RCLCPP_ERROR(node->get_logger(), "Joint %s: max_accelerations must be finite and > 0.0", joint_names_[i].c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      if (!std::isfinite(max_position_error_[i]) || max_position_error_[i] <= 0.0) {
        RCLCPP_ERROR(node->get_logger(), "Joint %s: max_position_error must be finite and > 0.0", joint_names_[i].c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      if (!std::isfinite(p_gains_[i]) || p_gains_[i] < 0.0 ||
          !std::isfinite(i_gains_[i]) || i_gains_[i] < 0.0 ||
          !std::isfinite(d_gains_[i]) || d_gains_[i] < 0.0 ||
          !std::isfinite(ff_gains_[i]) || ff_gains_[i] < 0.0) {
        RCLCPP_ERROR(node->get_logger(), "Joint %s: Gains must be finite and non-negative", joint_names_[i].c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
    }

    // Safe command buffer initialization
    CommandData init_cmd;
    init_cmd.velocities.assign(n_joints_, 0.0);
    init_cmd.timestamp = std::chrono::steady_clock::now();
    init_cmd.received = false;
    commands_buffer_.writeFromNonRT(init_cmd);

    // State buffer allocations
    last_position_cmd_.assign(n_joints_, 0.0);
    last_commanded_vel_.assign(n_joints_, 0.0);
    integrator_.assign(n_joints_, 0.0);
    prev_error_.assign(n_joints_, 0.0);
    filtered_vel_.assign(n_joints_, 0.0);
    joints_urdf_.resize(n_joints_);

    // Zero-allocation diagnostics publisher pre-allocation
    auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("~/controller_state", 1);
    pub_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(publisher);
    pub_->msg_.data.resize(n_joints_, 0.0);

    // Parse URDF limits if available
    if (node->has_parameter("robot_description")) {
      urdf_ = std::make_shared<urdf::Model>();
      std::string urdf_string = node->get_parameter("robot_description").as_string();
      if (urdf_->initString(urdf_string)) {
        for (size_t i = 0; i < n_joints_; i++) {
          joints_urdf_[i] = urdf_->getJoint(joint_names_[i]);
        }
        RCLCPP_INFO(node->get_logger(), "Successfully parsed TIAGo robot_description for joint limits.");
      } else {
        RCLCPP_WARN(node->get_logger(), "Failed to parse robot_description string. Joint limits disabled.");
      }
    } else {
      RCLCPP_INFO(node->get_logger(), "robot_description parameter not detected on node. Joint limits disabled.");
    }

    sub_command_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/command", rclcpp::SystemDefaultsQoS(),
      std::bind(&TiagoVelocityController::commandCB, this, std::placeholders::_1));

    counter_ = 0;
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    for (size_t i = 0; i < n_joints_; i++) {
      double pos = state_interfaces_[i * 2].get_value();
      last_position_cmd_[i] = pos;
      last_commanded_vel_[i] = 0.0;
      integrator_[i] = 0.0;
      prev_error_[i] = 0.0;
      filtered_vel_[i] = 0.0;
      command_interfaces_[i].set_value(pos);
    }

    // Invalidate stale references on activation
    CommandData reset_cmd;
    reset_cmd.velocities.assign(n_joints_, 0.0);
    reset_cmd.timestamp = std::chrono::steady_clock::now();
    reset_cmd.received = false;
    commands_buffer_.writeFromNonRT(reset_cmd);

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &period) override
  {
    const auto &cmd_data = *commands_buffer_.readFromRT();
    double dt = period.seconds();

    // Guard against negative time steps
    if (dt <= 0.0) {
      return controller_interface::return_type::OK;
    }

    // Scheduler jitter guard: clamp large execution delays to 50 ms max
    constexpr double MAX_ALLOWED_DT = 0.05;
    if (dt > MAX_ALLOWED_DT) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Jitter spike detected (dt = %f s). Clamped to %f s.", dt, MAX_ALLOWED_DT);
      dt = MAX_ALLOWED_DT;
    }

    // Safe timeout check using steady_clock
    const auto now = std::chrono::steady_clock::now();
    const bool is_timed_out = !cmd_data.received ||
      (std::chrono::duration<double>(now - cmd_data.timestamp).count() > cmd_timeout_sec_);

    for (size_t i = 0; i < n_joints_; i++) {
      double current_pos = state_interfaces_[i * 2].get_value();
      double current_vel = state_interfaces_[i * 2 + 1].get_value();

      // State interface validation
      if (!std::isfinite(current_pos) || !std::isfinite(current_vel)) {
        RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Invalid non-finite hardware state on joint %s (pos=%f, vel=%f). Halting.",
          joint_names_[i].c_str(), current_pos, current_vel);
        return controller_interface::return_type::ERROR;
      }

      // Tracking error fault check (|x_cmd - x_meas|)
      double tracking_error = std::abs(last_position_cmd_[i] - current_pos);
      if (tracking_error > max_position_error_[i]) {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Tracking error fault on joint %s: |cmd - meas| = %f rad (limit: %f rad). Tripping controller safe halt.",
          joint_names_[i].c_str(), tracking_error, max_position_error_[i]);
        last_position_cmd_[i] = current_pos;
        last_commanded_vel_[i] = 0.0;
        integrator_[i] = 0.0;
        command_interfaces_[i].set_value(current_pos);
        return controller_interface::return_type::ERROR;
      }

      // Filter feedback velocity
      filtered_vel_[i] = filter_alpha_ * current_vel + (1.0 - filter_alpha_) * filtered_vel_[i];

      // Safe target reference selection
      double target_vel = is_timed_out ? 0.0 : cmd_data.velocities[i];

      // Velocity limiting
      double clamped_vel = std::clamp(target_vel, -max_velocities_[i], max_velocities_[i]);

      // Acceleration slew-rate limiting (ramps down smoothly on timeout or abrupt steps)
      double max_delta_v = max_accelerations_[i] * dt;
      double ramped_vel = std::clamp(clamped_vel,
                                     last_commanded_vel_[i] - max_delta_v,
                                     last_commanded_vel_[i] + max_delta_v);
      last_commanded_vel_[i] = ramped_vel;

      // Decoupled zero-velocity holding: lock setpoint immediately once ramped velocity reaches 0
      if (std::abs(ramped_vel) < 1e-6) {
        last_commanded_vel_[i] = 0.0;
        integrator_[i] = 0.0;
        prev_error_[i] = 0.0;
        command_interfaces_[i].set_value(last_position_cmd_[i]);
      } else {
        double error = ramped_vel - filtered_vel_[i];
        double new_integrator = integrator_[i] + error * dt;

        // Position setpoint generation with consistent time scaling
        double delta_pos = (ramped_vel * dt * ff_gains_[i])
                         + (error * dt * p_gains_[i])
                         + (new_integrator * dt * i_gains_[i])
                         + ((error - prev_error_[i]) * dt * d_gains_[i]);

        double next_pos = last_position_cmd_[i] + delta_pos;

        // Joint limit clamping: only apply to REVOLUTE and PRISMATIC; bypass CONTINUOUS
        if (joints_urdf_[i] && joints_urdf_[i]->limits &&
            (joints_urdf_[i]->type == urdf::Joint::REVOLUTE || joints_urdf_[i]->type == urdf::Joint::PRISMATIC)) {
          if (next_pos > joints_urdf_[i]->limits->upper) {
            next_pos = joints_urdf_[i]->limits->upper;
            if (error < 0.0) integrator_[i] = new_integrator;
          } else if (next_pos < joints_urdf_[i]->limits->lower) {
            next_pos = joints_urdf_[i]->limits->lower;
            if (error > 0.0) integrator_[i] = new_integrator;
          } else {
            integrator_[i] = new_integrator;
          }
        } else {
          integrator_[i] = new_integrator;
        }

        // Hardware command numerical guard
        if (!std::isfinite(next_pos)) {
          RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "Non-finite position calculated for joint %s.", joint_names_[i].c_str());
          return controller_interface::return_type::ERROR;
        }

        command_interfaces_[i].set_value(next_pos);
        prev_error_[i] = error;
        last_position_cmd_[i] = next_pos;
      }
    }

    // Diagnostics publishing at ~100 Hz (zero heap allocation)
    if (counter_++ % 10 == 0 && pub_ && pub_->trylock()) {
      std::copy(filtered_vel_.begin(), filtered_vel_.end(), pub_->msg_.data.begin());
      pub_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
  }

private:
  std::vector<std::string> joint_names_;
  size_t n_joints_{0};
  uint64_t counter_{0};

  std::vector<double> last_position_cmd_;
  std::vector<double> last_commanded_vel_;
  std::vector<double> integrator_;
  std::vector<double> prev_error_;
  std::vector<double> filtered_vel_;
  std::vector<double> p_gains_;
  std::vector<double> i_gains_;
  std::vector<double> d_gains_;
  std::vector<double> ff_gains_;
  std::vector<double> max_velocities_;
  std::vector<double> max_accelerations_;
  std::vector<double> max_position_error_;
  double cmd_timeout_sec_{0.2};
  double filter_alpha_{0.95};

  std::vector<urdf::JointConstSharedPtr> joints_urdf_;
  std::shared_ptr<urdf::Model> urdf_;

  realtime_tools::RealtimeBuffer<CommandData> commands_buffer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_command_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> pub_;

  void commandCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != n_joints_) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Command vector size (%zu) != joint count (%zu). Dropping.",
                           msg->data.size(), n_joints_);
      return;
    }

    for (size_t i = 0; i < n_joints_; ++i) {
      if (!std::isfinite(msg->data[i])) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                             "Command contains non-finite values (NaN/Inf) at index %zu. Dropping.", i);
        return;
      }
    }

    CommandData data;
    data.velocities = msg->data;
    data.timestamp = std::chrono::steady_clock::now();
    data.received = true;
    commands_buffer_.writeFromNonRT(data);
  }
};

}  // namespace tiago_velocity_controller_ns

PLUGINLIB_EXPORT_CLASS(
  tiago_velocity_controller_ns::TiagoVelocityController,
  controller_interface::ControllerInterface)