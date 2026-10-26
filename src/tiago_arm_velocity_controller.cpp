#include <vector>
#include <string>
#include <memory>
#include <algorithm>

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

    // Declare standard ROS 2 parameters
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

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    auto node = get_node();

    joint_names_ = node->get_parameter("joints").as_string_array();
    n_joints_ = joint_names_.size();

    if (n_joints_ == 0) {
      RCLCPP_ERROR(node->get_logger(), "Joints parameter is empty!");
      return controller_interface::CallbackReturn::ERROR;
    }

    p_gains_ = node->get_parameter("p_gains").as_double_array();
    i_gains_ = node->get_parameter("i_gains").as_double_array();
    d_gains_ = node->get_parameter("d_gains").as_double_array();
    ff_gains_ = node->get_parameter("feedforward_gains").as_double_array();

    if (p_gains_.size() != n_joints_ || i_gains_.size() != n_joints_ ||
        d_gains_.size() != n_joints_ || ff_gains_.size() != n_joints_) {
      RCLCPP_ERROR(node->get_logger(), "Gains arrays dimensions do not match joint count (%zu)", n_joints_);
      return controller_interface::CallbackReturn::ERROR;
    }

    // Initialize state buffers
    commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
    last_position_cmd_ = std::vector<double>(n_joints_, 0.0);
    integrator_ = std::vector<double>(n_joints_, 0.0);
    prev_error_ = std::vector<double>(n_joints_, 0.0);
    filtered_vel_ = std::vector<double>(n_joints_, 0.0);
    joints_urdf_.resize(n_joints_);

    // Subscriber
    sub_command_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/command", rclcpp::SystemDefaultsQoS(),
      std::bind(&TiagoVelocityController::commandCB, this, std::placeholders::_1));

    // Realtime publisher for state diagnostics
    auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("~/controller_state", 1);
    pub_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(publisher);

    // Optional URDF extraction for TIAGo joint limits
    if (node->has_parameter("robot_description")) {
      urdf_ = std::make_shared<urdf::Model>();
      std::string urdf_string = node->get_parameter("robot_description").as_string();
      if (urdf_->initString(urdf_string)) {
        for (size_t i = 0; i < n_joints_; i++) {
          joints_urdf_[i] = urdf_->getJoint(joint_names_[i]);
        }
        RCLCPP_INFO(node->get_logger(), "Successfully parsed TIAGo robot_description for limits.");
      } else {
        RCLCPP_WARN(node->get_logger(), "Failed to parse robot_description. Clamping disabled.");
      }
    } else {
      RCLCPP_INFO(node->get_logger(), "No robot_description parameter detected. Clamping disabled.");
    }

    counter_ = 0;
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    for (size_t i = 0; i < n_joints_; i++) {
      // Index mapping: position = i * 2, velocity = i * 2 + 1
      double pos = state_interfaces_[i * 2].get_value();
      last_position_cmd_[i] = pos;
      integrator_[i] = 0.0;
      prev_error_[i] = 0.0;
      filtered_vel_[i] = 0.0;
      command_interfaces_[i].set_value(pos);
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &period) override
  {
    auto &commands = *commands_buffer_.readFromRT();
    double dt = period.seconds();

    if (dt <= 0.0) {
      return controller_interface::return_type::OK;
    }

    for (size_t i = 0; i < n_joints_; i++) {
      double vel_cmd = commands[i];
      double current_pos = state_interfaces_[i * 2].get_value();
      double current_vel = state_interfaces_[i * 2 + 1].get_value();

      // Low-pass filter velocity feedback (cutoff tuning)
      filtered_vel_[i] = 0.95 * current_vel + 0.05 * filtered_vel_[i];

      if (std::abs(vel_cmd) < 1e-6) {
        integrator_[i] = 0.0;
        prev_error_[i] = 0.0;
        // Hold current position setpoint
        command_interfaces_[i].set_value(last_position_cmd_[i]);
      } else {
        double error = vel_cmd - filtered_vel_[i];
        double new_integrator = integrator_[i] + error * dt;

        // Position integration via Feedforward + PID
        double next_pos = current_pos + vel_cmd * dt * ff_gains_[i]
                          + error * p_gains_[i]
                          + new_integrator * i_gains_[i]
                          + (d_gains_[i] / dt) * (error - prev_error_[i]);

        // Anti-windup conditional integration against joint limits
        if (joints_urdf_[i] && joints_urdf_[i]->limits) {
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

        command_interfaces_[i].set_value(next_pos);
        prev_error_[i] = error;
        last_position_cmd_[i] = next_pos;
      }
    }

    if (counter_++ % 10 == 0 && pub_ && pub_->trylock()) {
      pub_->msg_.data = filtered_vel_;
      pub_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
  }

private:
  std::vector<std::string> joint_names_;
  size_t n_joints_{0};
  uint64_t counter_{0};

  std::vector<double> last_position_cmd_;
  std::vector<double> integrator_;
  std::vector<double> prev_error_;
  std::vector<double> filtered_vel_;
  std::vector<double> p_gains_;
  std::vector<double> i_gains_;
  std::vector<double> d_gains_;
  std::vector<double> ff_gains_;

  std::vector<urdf::JointConstSharedPtr> joints_urdf_;
  std::shared_ptr<urdf::Model> urdf_;

  realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_command_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> pub_;

  void commandCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != n_joints_) {
      return;
    }
    commands_buffer_.writeFromNonRT(msg->data);
  }
};

}  // namespace tiago_velocity_controller_ns

PLUGINLIB_EXPORT_CLASS(
  tiago_velocity_controller_ns::TiagoVelocityController,
  controller_interface::ControllerInterface)