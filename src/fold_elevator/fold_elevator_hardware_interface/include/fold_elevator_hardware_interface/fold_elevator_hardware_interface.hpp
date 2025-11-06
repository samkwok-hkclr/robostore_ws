
#ifndef FOLD_ELEVATOR_HARDWARE_INTERFACE_HPP__
#define FOLD_ELEVATOR_HARDWARE_INTERFACE_HPP__

#pragma once

#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/macros.hpp>

#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <can_msgs/msg/frame.hpp>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include "fold_elevator_hardware_interface/Ti5Robot_cra_driver.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace fold_elevator_hardware_interface {

// 电机参数结构
struct MotorConfig 
{
  uint8_t can_id;
  double gear_ratio;
  int32_t position_offset;
  int32_t position_p;
  int32_t position_d;
  int32_t velocity_p;
  int32_t velocity_i;
  int32_t max_current;
  int32_t min_current;
  double max_velocity;
  double min_velocity;
};

class FoldElevatorHardwareInterface : public hardware_interface::SystemInterface
{
  using UInt8 = std_msgs::msg::UInt8;
  using Trigger = std_srvs::srv::Trigger;
  using SetBool = std_srvs::srv::SetBool;

public:
  FoldElevatorHardwareInterface();
  virtual ~FoldElevatorHardwareInterface() = default;
  
  RCLCPP_SHARED_PTR_DEFINITIONS(FoldElevatorHardwareInterface)
  
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo& info) override;

  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> 
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> 
  export_command_interfaces() override;

  hardware_interface::return_type 
  read(const rclcpp::Time & time, const rclcpp::Duration& period) override;
  
  hardware_interface::return_type 
  write(const rclcpp::Time & time, const rclcpp::Duration& period) override;

  void executor_loop(void);
  void lock_status_pub_cb(void);
  void auto_halt_cb(void);
  void action_command_cb(
    const std::shared_ptr<SetBool::Request> request, 
    std::shared_ptr<SetBool::Response> response);

private:
  rclcpp::Logger logger_;
  std::atomic<bool> activated_;

  std::atomic<bool> shutdown_requested_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<rclcpp::Node> node_;

  rclcpp::TimerBase::SharedPtr lock_status_pub_timer_;
  rclcpp::TimerBase::SharedPtr auto_halt_timer_;

  rclcpp::Publisher<UInt8>::SharedPtr lock_status_pub_;

  rclcpp::Service<SetBool>::SharedPtr action_command_srv_;
  std::thread executor_thread_;
  
  std::unordered_map<std::string, size_t> joint_indices_;
  
  std::vector<double> hw_position_commands_;
  std::vector<double> hw_velocity_commands_;
  std::vector<double> hw_effort_commands_;
  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;
  std::vector<double> hw_effort_states_;

  constexpr static size_t NUM_HISTORY_POSITION = 100;
  std::mutex halt_mutex_;
  std::mutex state_mutex_;
  std::vector<std::deque<double>> position_trackers_;
  std::vector<double> running_sums_;
  std::vector<MotorState> position_status_;
  std::vector<LockState> lock_status_;
  bool action_command_;
  bool halt_timer_activated_;
  
  constexpr static float RAD_DIFF_THRESHOLD = 0.005f; // To be determine
  constexpr static int32_t POSITION_CHANGE_THRESHOLD = 0x7F;

  // 标志变量，指示是否支持某种命令接口
  std::vector<bool> supports_position_command_;
  std::vector<bool> supports_velocity_command_;
  std::vector<bool> supports_effort_command_;

  // 电机配置
  std::vector<MotorConfig> motor_configs_;

  std::string can_interface_;  
  std::unique_ptr<Ti5RobotCRADriver> motorDriver_;

  // 常量
  static constexpr double ENCODER_RESOLUTION = 262144.0; // 18-bit encoder
  static constexpr double VELOCITY_SCALE = 100.0;
  static constexpr double RAD_PER_REV = 2 * M_PI;

  void update_motor_state(size_t index, uint8_t can_id, double new_position);

  double position_convention(int32_t val) const;
  int32_t position_inverse_convention(double val) const;
  double velocity_convention(int32_t val, size_t joint_index) const;
  int32_t velocity_inverse_convention(double val, size_t joint_index) const;
  
};  // class FoldElevatorHardwareInterface

} // namespace fold_elevator_hardware_interface

#endif // FOLD_ELEVATOR_HARDWARE_INTERFACE_HPP__