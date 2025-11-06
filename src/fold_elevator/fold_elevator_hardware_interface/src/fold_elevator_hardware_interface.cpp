#include "fold_elevator_hardware_interface/fold_elevator_hardware_interface.hpp"
#include "fold_elevator_hardware_interface/Ti5Robot_cra_driver.hpp"

namespace fold_elevator_hardware_interface {

FoldElevatorHardwareInterface::FoldElevatorHardwareInterface() 
  : hardware_interface::SystemInterface()
  , logger_(rclcpp::get_logger("FoldElevatorHardwareInterface"))
  , activated_(false)
  , shutdown_requested_(false)
  , executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
  , node_(std::make_shared<rclcpp::Node>("auxiliary_node"))
  , action_command_(true)
  , halt_timer_activated_(false)
{
  lock_status_pub_ = node_->create_publisher<UInt8>("fold_elevator_lock_status", 10);

  lock_status_pub_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(500), 
    std::bind(&FoldElevatorHardwareInterface::lock_status_pub_cb, this));

  // auto_halt_timer_ = node_->create_wall_timer(
  //   std::chrono::milliseconds(1000), 
  //   std::bind(&FoldElevatorHardwareInterface::auto_halt_cb, this));
  // auto_halt_timer_->cancel();

  action_command_srv_ = node_->create_service<SetBool>(
    "fold_elevator_action", 
    std::bind(&FoldElevatorHardwareInterface::action_command_cb, this, _1, _2),
    rmw_qos_profile_services_default);

  executor_->add_node(node_);
  executor_thread_ = std::thread(std::bind(&FoldElevatorHardwareInterface::executor_loop, this));

  RCLCPP_INFO(logger_, "FoldElevatorHardwareInterface initialized");
}

hardware_interface::CallbackReturn 
FoldElevatorHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "to initialize %zu motors", info.joints.size());

  joint_indices_.reserve(info.joints.size());
  supports_position_command_.resize(info.joints.size(), false);
  supports_velocity_command_.resize(info.joints.size(), false);
  supports_effort_command_.resize(info.joints.size(), false);

  motor_configs_.resize(info.joints.size());
  std::vector<struct can_filter> can_filters;

  for (size_t i = 0; i < info.joints.size(); ++i) {
    const auto& joint = info.joints[i];
    joint_indices_[joint.name] = i;

    for (const auto& cmd_if : joint.command_interfaces) {
      if (cmd_if.name == hardware_interface::HW_IF_POSITION) {
        supports_position_command_[i] = true;
      } else if (cmd_if.name == hardware_interface::HW_IF_VELOCITY) {
        supports_velocity_command_[i] = true;
      } else if (cmd_if.name == hardware_interface::HW_IF_EFFORT) {
        supports_effort_command_[i] = true;
      }
    }
    
    try {
      motor_configs_[i].can_id = static_cast<uint32_t>(std::stoi(joint.parameters.at("can_id"), nullptr, 16));
      can_filters.push_back({motor_configs_[i].can_id, CAN_SFF_MASK});
      motor_configs_[i].gear_ratio = std::stod(joint.parameters.at("gear_ratio"));
      motor_configs_[i].position_offset = joint.parameters.count("position_offset") ? 
              static_cast<int32_t>(std::stol(joint.parameters.at("position_offset"), nullptr, 16)) : 0 ; // 支持16进制格式

      RCLCPP_INFO(logger_, "can_id: 0x%x, gear_ratio: %f, position_offset: 0x%08x", 
                    motor_configs_[i].can_id, motor_configs_[i].gear_ratio, motor_configs_[i].position_offset);
      // PID参数
      motor_configs_[i].position_p = joint.parameters.count("position_p") ? std::stoi(joint.parameters.at("position_p")) : 1000;
      motor_configs_[i].position_d = joint.parameters.count("position_d") ? std::stoi(joint.parameters.at("position_d")) : 500;
      motor_configs_[i].velocity_p = joint.parameters.count("velocity_p") ? std::stoi(joint.parameters.at("velocity_p")) : 300;
      motor_configs_[i].velocity_i = joint.parameters.count("velocity_i") ? std::stoi(joint.parameters.at("velocity_i")) : 500;
      
      motor_configs_[i].max_current = joint.parameters.count("max_current") ? std::stoi(joint.parameters.at("max_current")) : 10000;
      motor_configs_[i].min_current = joint.parameters.count("min_current") ? std::stoi(joint.parameters.at("min_current")) : -10000;
      
      motor_configs_[i].max_velocity = joint.parameters.count("max_velocity") ? std::stod(joint.parameters.at("max_velocity")) : 0.5;
      motor_configs_[i].min_velocity = joint.parameters.count("min_velocity") ? std::stod(joint.parameters.at("min_velocity")) : -0.5;
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger_, "Failed to initialize joint %s: %s", joint.name.c_str(), e.what());
      return CallbackReturn::ERROR;
    }
  }

  hw_position_states_.resize(info.joints.size(), std::numeric_limits<double>::max());
  hw_velocity_states_.resize(info.joints.size(), 0.0);
  hw_effort_states_.resize(info.joints.size(), 0.0);
  
  hw_position_commands_.resize(info.joints.size(), 0.0);
  hw_velocity_commands_.resize(info.joints.size(), 0.0);
  hw_effort_commands_.resize(info.joints.size(), 0.0);

  lock_status_.resize(info.joints.size(), LockState::LOCKED);
  position_trackers_.resize(info.joints.size(), {});
  running_sums_.resize(info.joints.size(), 0.0);
  position_status_.resize(info.joints.size(), MotorState::IDLE);

  if (info.hardware_parameters.count("can_interface")) {
    can_interface_ = info.hardware_parameters.at("can_interface");
  } else {
    return CallbackReturn::ERROR;
  }
  
  motorDriver_ = std::make_unique<Ti5RobotCRADriver>(can_interface_);
  motorDriver_->setCanFrameFilters(can_filters);
  motorDriver_->setCanTimeout(0.5, 0.5);

  RCLCPP_INFO(logger_, "Successfully initialize %zu motors", motor_configs_.size());

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn 
FoldElevatorHardwareInterface::on_configure(const rclcpp_lifecycle::State& /* previous_state */)
{
  for (size_t i = 0; i < motor_configs_.size(); ++i) {
    const auto& config = motor_configs_[i];
    if (config.can_id == 0 || config.gear_ratio <= 0) {
      RCLCPP_ERROR(logger_, "Invalid configuration for motor %zu", i);
      return CallbackReturn::ERROR;
    }
    
    Ti5RobotCRADriverStatus motor_status = Ti5RobotCRADriverStatus::SUCCESS;
    
    motor_status = motorDriver_->setPositionP(config.can_id, config.position_p);
    if (motor_status != Ti5RobotCRADriverStatus::SUCCESS) {
      RCLCPP_ERROR(logger_, "Error(%d) in calling motorDriver_->setPositionP(0x%x, %d)", 
                        motor_status, config.can_id, config.position_p);
      return CallbackReturn::ERROR;
    }

    motor_status = motorDriver_->setPositionD(config.can_id, config.position_d);
    if (motor_status != Ti5RobotCRADriverStatus::SUCCESS) {
      RCLCPP_ERROR(logger_, "Error(%d) in calling motorDriver_->setPositionD(0x%x, %d)", 
                        motor_status, config.can_id, config.position_d);
      return CallbackReturn::ERROR;
    }

    motor_status = motorDriver_->setVelocityP(config.can_id, config.velocity_p);
    if (motor_status != Ti5RobotCRADriverStatus::SUCCESS) {
      RCLCPP_ERROR(logger_, "Error(%d) in calling motorDriver_->setVelocityP(0x%x, %d)", 
                        motor_status, config.can_id, config.velocity_p);
      return CallbackReturn::ERROR;
    }

    motor_status = motorDriver_->setVelocityI(config.can_id, config.velocity_i);
    if (motor_status != Ti5RobotCRADriverStatus::SUCCESS) {
      RCLCPP_ERROR(logger_, "Error(%d) in calling motorDriver_->velocity_i(0x%x, %d)", 
                        motor_status, config.can_id, config.velocity_i);
      return CallbackReturn::ERROR;
    }

    motor_status = motorDriver_->setMaxCurrent(config.can_id, config.max_current);
    if (motor_status != Ti5RobotCRADriverStatus::SUCCESS) {
      RCLCPP_ERROR(logger_, "Error(%d) in calling motorDriver_->setMaxCurrent(0x%x, %d)", 
                        motor_status, config.can_id, config.max_current);
      return CallbackReturn::ERROR;
    }

    motor_status = motorDriver_->setMinCurrent(config.can_id, config.min_current);
    if (motor_status != Ti5RobotCRADriverStatus::SUCCESS) {
      RCLCPP_ERROR(logger_, "Error(%d) in calling motorDriver_->setMinCurrent(0x%x, %d)", 
                        motor_status, config.can_id, config.min_current);
      return CallbackReturn::ERROR;
    }

    int32_t converted_max_velocity = velocity_inverse_convention(config.max_velocity, i);
    int32_t converted_min_velocity = velocity_inverse_convention(config.min_velocity, i);

    motor_status = motorDriver_->setMaxVelocity(config.can_id, converted_max_velocity);
    if (motor_status != Ti5RobotCRADriverStatus::SUCCESS) {
      RCLCPP_ERROR(logger_, "Error(%d) in calling motorDriver_->setMaxVelocity(0x%x, 0x%08x)", 
                  motor_status, config.can_id, converted_max_velocity);
      return CallbackReturn::ERROR;
    }

    motor_status = motorDriver_->setMinVelocity(config.can_id, converted_min_velocity);
    if (motor_status != Ti5RobotCRADriverStatus::SUCCESS) {
      RCLCPP_ERROR(logger_, "Error(%d) in calling motorDriver_->setMinVelocity(0x%x, 0x%08x)", 
                  motor_status, config.can_id, converted_min_velocity);
      return CallbackReturn::ERROR;
    }

    motor_status = motorDriver_->setPositionOffset(config.can_id, config.position_offset);
    if (motor_status != Ti5RobotCRADriverStatus::SUCCESS) {
      RCLCPP_ERROR(logger_, "Error(%d) in calling motorDriver_->setPositionOffset(0x%x, 0x%08x)", 
                    motor_status, config.can_id, config.position_offset);
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(logger_, "Successfully configured %zu motors", motor_configs_.size());

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
FoldElevatorHardwareInterface::on_activate(const rclcpp_lifecycle::State& /* previous_state */)
{
  if (activated_) {
    RCLCPP_FATAL(logger_, "Double on_activate()");
    return CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < motor_configs_.size(); ++i) {
    const auto& config = motor_configs_[i];
    Ti5RobotCRADriverStatus motor_status;
    int32_t value = 0;

    motor_status = motorDriver_->getPosition(config.can_id, value);
    if (motor_status != Ti5RobotCRADriverStatus::SUCCESS) {
      RCLCPP_ERROR(logger_, "Error (%d) in calling motorDriver_->getPosition(0x%x, 0x%08x)", motor_status, config.can_id, value);
      return CallbackReturn::ERROR;
    }

    const double tmp = position_convention(value);
    hw_position_states_[i] = tmp;

    // motor_status = motorDriver_->activateWithPositionMode(config.can_id, value);
    // if (motor_status != Ti5RobotCRADriverStatus::SUCCESS) {
    //   RCLCPP_ERROR(logger_, "Error(%d) in calling motorDriver_->activateWithPositionMode(0x%x, 0x%08x)", motor_status, config.can_id, value);
    //   return CallbackReturn::ERROR;
    // }    

    hw_position_commands_[i] = tmp;

    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  RCLCPP_INFO(logger_, "Successfully activated!");

  activated_ = true;
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
FoldElevatorHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /* previous_state*/)
{
  // 停止所有电机
  for (const auto& config : motor_configs_) {
    motorDriver_->deactivate(config.can_id);
  }

  activated_ = false;
  RCLCPP_INFO(logger_, "Successfully deactivated!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn 
FoldElevatorHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& /* previous_state */)
{
  shutdown_requested_.store(true);
  
  if (executor_thread_.joinable()) 
    executor_thread_.join();

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
FoldElevatorHardwareInterface::on_shutdown(const rclcpp_lifecycle::State& /* previous_state */)
{
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> 
FoldElevatorHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (const auto& joint : info_.joints) {
    size_t index = joint_indices_.at(joint.name);
    
    // 检查并添加位置状态接口
    for (const auto& state_if : joint.state_interfaces) {
      if (state_if.name == hardware_interface::HW_IF_POSITION) {
        state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &hw_position_states_[index]);
      } else if (state_if.name == hardware_interface::HW_IF_VELOCITY) {
        state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[index]);
      } else if (state_if.name == hardware_interface::HW_IF_EFFORT) {
        state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &hw_effort_states_[index]);
      }
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> 
FoldElevatorHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (const auto& joint : info_.joints) {
    size_t index = joint_indices_.at(joint.name);
    for (const auto& cmd_if : joint.command_interfaces) {
      if (cmd_if.name == hardware_interface::HW_IF_POSITION) {
        command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[index]);
      } else if (cmd_if.name == hardware_interface::HW_IF_VELOCITY) {
        command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[index]);
      } else if (cmd_if.name == hardware_interface::HW_IF_EFFORT) {
        command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &hw_effort_commands_[index]);
      }
    }
  }

  return command_interfaces;
}

hardware_interface::return_type 
FoldElevatorHardwareInterface::read(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
{
  if (!activated_)
    return hardware_interface::return_type::OK;

  int can_error = 0;
   
  for (const auto& joint : info_.joints) {
    size_t index = joint_indices_.at(joint.name);
    uint8_t can_id = motor_configs_[index].can_id;

    int32_t value = 0;
    Ti5RobotCRADriverStatus motor_status = motorDriver_->getPosition(can_id, value);
    if (motor_status != Ti5RobotCRADriverStatus::SUCCESS) {
      RCLCPP_ERROR(logger_, "Error (%d) in calling motorDriver_->getPosition(0x%x, 0x%08x)", motor_status, can_id, value);
      can_error = 1;
      break;
    } 

    const double tmp = position_convention(value);

    hw_position_states_[index] = tmp;
    update_motor_state(index, can_id, tmp);
  }

  return (!can_error ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR);
}

hardware_interface::return_type 
FoldElevatorHardwareInterface::write(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
{
  if (!activated_) 
    return hardware_interface::return_type::OK;

  {
    std::lock_guard<std::mutex> lock(halt_mutex_);
    if (!action_command_)
      return hardware_interface::return_type::OK;
  }

  int can_error = 0;
  
  for (const auto& joint : info_.joints) {
    const int index = joint_indices_.at(joint.name);

    if (supports_position_command_[index] && !std::isnan(hw_position_commands_[index])) {
      // 位置控制模式
      const uint8_t can_id = motor_configs_[index].can_id;
      const int32_t value = position_inverse_convention(hw_position_commands_[index]);

      Ti5RobotCRADriverStatus motor_status = motorDriver_->setTargetPosition(can_id, value);
      if (motor_status != Ti5RobotCRADriverStatus::SUCCESS) {
        RCLCPP_ERROR(logger_, "Error (%d) in calling motorDriver_->setTargetPosition(0x%x, 0x%08x)", motor_status, can_id, value);
        can_error = 1;
        break;
      }
      else
      {
        std::lock_guard<std::mutex> lock(halt_mutex_);
        lock_status_[index] = LockState::UNLOCKED;
      }

      RCLCPP_DEBUG(logger_, "Set position mode 0x%x, value: 0x%08x", can_id, value);
    }
  }
  
  return (!can_error ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR);
}

void FoldElevatorHardwareInterface::lock_status_pub_cb(void)
{
  std_msgs::msg::UInt8 msg;
  msg.data = 0;
  
  /*
    0000 (0) = All motors unlocked
    1010 (10) = Motors 1 and 3 locked, others unlocked
    1111 (15) = All motors locked
  */
  {
    std::lock_guard<std::mutex> lock(halt_mutex_);

    for (size_t i = 0; i < info_.joints.size(); ++i) 
    {
      const auto& joint = info_.joints[i];
      const int index = joint_indices_.at(joint.name);
      
      if (lock_status_[index] == LockState::LOCKED) 
        msg.data |= (1 << i);  // Set the i-th bit (0-3)
    }
  }

  lock_status_pub_->publish(msg);
}

void FoldElevatorHardwareInterface::auto_halt_cb(void)
{
  std::lock_guard<std::mutex> lock(halt_mutex_);

  if (!action_command_)
    return;

  RCLCPP_DEBUG(logger_, "Try to lock motor now");

  for (const auto& joint : info_.joints) {
    const int index = joint_indices_.at(joint.name);

    if (position_status_[index] != MotorState::IDLE)
      continue;
    
    if (lock_status_[index] == LockState::LOCKED) 
      continue;
    
    const uint8_t can_id = motor_configs_[index].can_id;

    Ti5RobotCRADriverStatus motor_status = motorDriver_->deactivate(can_id);

    if (motor_status == Ti5RobotCRADriverStatus::SUCCESS) {
      lock_status_[index] = LockState::LOCKED;
      RCLCPP_WARN(logger_, "Successfully locked motor 0x%x", can_id);
    } else {
      RCLCPP_ERROR(logger_, "Error (%d) in calling motorDriver_->deactivate(0x%x,)", motor_status, can_id);
    }
  }

  bool all_locked = true;

  for (const auto& joint : info_.joints) {
    const int index = joint_indices_.at(joint.name);
    if (lock_status_[index] != LockState::LOCKED) {
      all_locked = false;
      break;
    }
  }

  if (all_locked) {
    action_command_ = false;
    RCLCPP_ERROR(logger_, "All motors are locked");
  }

  // halt_timer_activated_ = false;
  // auto_halt_timer_->cancel();
  // RCLCPP_ERROR(logger_, "Auto halt timer turn-off.");
}

void FoldElevatorHardwareInterface::action_command_cb(
  const std::shared_ptr<SetBool::Request> request, 
  std::shared_ptr<SetBool::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(halt_mutex_);
    action_command_ = request->data; // always to assign true to action_command_
  }

  response->success = true;

  if (request->data)
  {
    // for (const auto& joint : info_.joints) {
    for (auto it = info_.joints.rbegin(); it != info_.joints.rend(); ++it) {
      const int index = joint_indices_.at(it->name);

      if (supports_position_command_[index] && !std::isnan(hw_position_commands_[index])) {
        // 位置控制模式
        const uint8_t can_id = motor_configs_[index].can_id;
        const int32_t value = position_inverse_convention(hw_position_commands_[index]);

        Ti5RobotCRADriverStatus motor_status = motorDriver_->setTargetPosition(can_id, value);
        if (motor_status != Ti5RobotCRADriverStatus::SUCCESS) {
          RCLCPP_ERROR(logger_, "Error (%d) in calling motorDriver_->setTargetPosition(0x%x, 0x%08x)", motor_status, can_id, value);
        }
        else
        {
          std::lock_guard<std::mutex> lock(halt_mutex_);
          lock_status_[index] = LockState::UNLOCKED;
          RCLCPP_WARN(logger_, "Set position mode 0x%x, value: 0x%08x", can_id, value);
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }
  else
  {
    for (const auto& joint : info_.joints) {
      const int index = joint_indices_.at(joint.name);
      const uint8_t can_id = motor_configs_[index].can_id;

      Ti5RobotCRADriverStatus motor_status = motorDriver_->deactivate(can_id);

      if (motor_status == Ti5RobotCRADriverStatus::SUCCESS) {
        std::lock_guard<std::mutex> lock(halt_mutex_);
        lock_status_[index] = LockState::LOCKED;
        RCLCPP_WARN(logger_, "Successfully locked motor 0x%x", can_id);
      } else {
        RCLCPP_ERROR(logger_, "Error (%d) in calling motorDriver_->deactivate(0x%x,)", motor_status, can_id);
      }
    } 
  }
}

void FoldElevatorHardwareInterface::update_motor_state(size_t index, uint8_t can_id, double new_position)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto& tracker = position_trackers_[index];

  if (tracker.size() >= NUM_HISTORY_POSITION) {
    tracker.pop_front();
  }

  tracker.push_back(new_position);

  if (tracker.size() == 0) {
    RCLCPP_ERROR(logger_, "This case should not be displayed");
    return;
  }

  const float mean = std::accumulate(tracker.begin(), tracker.end(), 0.0) / tracker.size();
  const float diff = std::abs(mean - new_position);
  
  if (diff < RAD_DIFF_THRESHOLD) {
    position_status_[index] = MotorState::IDLE;

    // if (lock_status_[index] != LockState::LOCKED) {
    //   if (!halt_timer_activated_) {
    //     auto_halt_timer_.reset();
    //     halt_timer_activated_ = true;
    //     RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 1000, "Started halt timer for joint (CAN ID: 0x%x)", can_id);
    //   }
    // }
    
    RCLCPP_DEBUG_THROTTLE(logger_, *node_->get_clock(), 1000, "Joint idle (CAN ID: 0x%x)", can_id);
  } else {
    position_status_[index] = MotorState::MOVING;
    RCLCPP_DEBUG_THROTTLE(logger_, *node_->get_clock(), 1000, "Joint moving (CAN ID: 0x%x, diff: %.6f)", can_id, diff);
  }
}


void FoldElevatorHardwareInterface::executor_loop(void)
{
  RCLCPP_INFO(logger_, "Start the executor loop");
  
  while (rclcpp::ok() && !shutdown_requested_.load())
  {
    executor_->spin_once();
  }
}

// 转换函数实现
double FoldElevatorHardwareInterface::position_convention(int32_t val) const
{
  return static_cast<double>(val * 1.0 / ENCODER_RESOLUTION * RAD_PER_REV);
}

int32_t FoldElevatorHardwareInterface::position_inverse_convention(double val) const
{
  return static_cast<int32_t>((val * ENCODER_RESOLUTION) / RAD_PER_REV);
}

double FoldElevatorHardwareInterface::velocity_convention(int32_t val, size_t joint_index) const
{
  return static_cast<double>(val) / VELOCITY_SCALE / motor_configs_[joint_index].gear_ratio * RAD_PER_REV;
}

int32_t FoldElevatorHardwareInterface::velocity_inverse_convention(double val, size_t joint_index) const
{
  return static_cast<int32_t>((val * motor_configs_[joint_index].gear_ratio * VELOCITY_SCALE) / RAD_PER_REV);
}

} // namespace fold_elevator_hardware_interface
  
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  fold_elevator_hardware_interface::FoldElevatorHardwareInterface,
  hardware_interface::SystemInterface
)
