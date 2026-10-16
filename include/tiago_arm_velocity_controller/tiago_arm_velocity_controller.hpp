#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <urdf/model.h>

namespace tiago_arm_controllers
{

class TiagoArmVelocityPositionController : public controller_interface::ControllerInterface
{
public:
  TiagoArmVelocityPositionController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct JointLimit {
    bool has_limits{false};
    double lower{0.0};
    double upper{0.0};
  };

  struct CommandData {
    std::vector<double> velocities;
    rclcpp::Time timestamp{0, 0, RCL_ROS_TIME};
  };

  std::vector<std::string> joint_names_;
  size_t n_joints_{0};

  // Safe Realtime Buffering
  realtime_tools::RealtimeBuffer<CommandData> commands_buffer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_command_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> pub_controller_state_;

  // State interface lookup indices
  std::vector<size_t> pos_state_indices_;
  std::vector<size_t> vel_state_indices_;

  // Control State & Gains (Original HSR structure)
  std::vector<double> p_gains_;
  std::vector<double> i_gains_;
  std::vector<double> d_gains_;
  std::vector<double> ff_gains_;
  std::vector<double> max_accelerations_;

  std::vector<double> js_;
  std::vector<double> old_integrator_;
  std::vector<double> old_error_;
  std::vector<double> filtered_vel_;
  std::vector<double> ramped_vel_cmd_;

  std::vector<JointLimit> joint_limits_;

  double timeout_seconds_{0.2};
  float filter_alpha_{0.95f};
  uint64_t update_counter_{0};
};

}  // namespace tiago_arm_controllers