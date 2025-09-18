#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "fold_elevator_hardware_interface/Ti5Robot_cra_driver.hpp"

Ti5RobotCRADriver::Ti5RobotCRADriver(const std::string interface)
{
    interface_ = interface;
    can_driver_ = std::make_unique<SocketCanDriver>(interface, false, false);

    sender_timeout_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>
                (std::chrono::duration<double>(default_sender_timeout_sec));

    receiver_timeout_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>
                (std::chrono::duration<double>(default_receiver_timeout_sec));
}

Ti5RobotCRADriver::~Ti5RobotCRADriver()
{
}

void Ti5RobotCRADriver::setCanTimeout(double sender_timeout_sec, double receiver_timeout_sec)
{
    if (sender_timeout_sec < 0 || receiver_timeout_sec < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("Ti5RobotCRADriver"), 
            "Invalid sender_timeout_sec : %f or invalid receiver_timeout_sec: %f", sender_timeout_sec, sender_timeout_sec);
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("Ti5RobotCRADriver"), "setCanTimeout(%f, %f)", sender_timeout_sec, receiver_timeout_sec);

    sender_timeout_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>
                (std::chrono::duration<double>(sender_timeout_sec));

    receiver_timeout_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>
                (std::chrono::duration<double>(receiver_timeout_sec));

    return;
}

void Ti5RobotCRADriver::setCanFrameFilters(std::vector<struct can_filter> filters)
{
    SocketCanDriver::CanFilterList filter_list;
    filter_list.filters = filters;

    for (auto filter : filters) {
        RCLCPP_INFO(rclcpp::get_logger("Ti5RobotCRADriver"), "can_id: 0x%08x, can_mask: 0x%08x", filter.can_id, filter.can_mask);
    }

    can_driver_->SetCanFilters(filter_list);
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::setPositionP(uint32_t can_id, int32_t value)
{
    RCLCPP_INFO(rclcpp::get_logger("Ti5RobotCRADriver"), "setPositionP(0x%x, 0x%08x)", can_id, value);
    // no can message return
    return sendCanData(can_id, MotorCommand::SET_POSITION_P, value);
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::setPositionD(uint32_t can_id, int32_t value)
{
    RCLCPP_INFO(rclcpp::get_logger("Ti5RobotCRADriver"), "setPositionD(0x%x, 0x%08x)", can_id, value);
    // no can message return
    return sendCanData(can_id, MotorCommand::SET_POSITION_D, value);
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::setVelocityP(uint32_t can_id, int32_t value)
{
    RCLCPP_INFO(rclcpp::get_logger("Ti5RobotCRADriver"), "setVelocityP(0x%x, 0x%08x)", can_id, value);
    // no can message return
    return sendCanData(can_id, MotorCommand::SET_VELOCITY_P, value);
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::setVelocityI(uint32_t can_id, int32_t value)
{
    RCLCPP_INFO(rclcpp::get_logger("Ti5RobotCRADriver"), "setVelocityI(0x%x, 0x%08x)", can_id, value);
    // no can message return
    return sendCanData(can_id, MotorCommand::SET_VELOCITY_I, value);
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::setMaxCurrent(uint32_t can_id, int32_t value)
{
    RCLCPP_INFO(rclcpp::get_logger("Ti5RobotCRADriver"), "setMaxCurrent(0x%x, 0x%08x)", can_id, value);
    // no can message return
    return sendCanData(can_id, MotorCommand::SET_MAX_POS_CURRENT, value);
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::setMinCurrent(uint32_t can_id, int32_t value)
{
    RCLCPP_INFO(rclcpp::get_logger("Ti5RobotCRADriver"), "setMinCurrent(0x%x, 0x%08x)", can_id, value);
    // no can message return
    return sendCanData(can_id, MotorCommand::SET_MIN_NEG_CURRENT, value);
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::setMaxVelocity(uint32_t can_id, int32_t value)
{
    RCLCPP_INFO(rclcpp::get_logger("Ti5RobotCRADriver"), "setMaxVelocity(0x%x, 0x%08x)", can_id, value);
    // no can message return
    return sendCanData(can_id, MotorCommand::SET_MAX_VELOCITY, value);
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::setMinVelocity(uint32_t can_id, int32_t value)
{
    RCLCPP_INFO(rclcpp::get_logger("Ti5RobotCRADriver"), "setMinVelocity(0x%x, 0x%08x)", can_id, value);
    // no can message return
    return sendCanData(can_id, MotorCommand::SET_MIN_VELOCITY, value);
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::setPositionOffset(uint32_t can_id, int32_t value)
{
    RCLCPP_INFO(rclcpp::get_logger("Ti5RobotCRADriver"), "setPositionOffset(0x%x, 0x%08x)", can_id, value);
    // no can message return
    return sendCanData(can_id, MotorCommand::SET_POSITION_OFFSET, value);
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::activateWithPositionMode(uint32_t can_id, int32_t currentPosition)
{
    // Ti5RobotCRADriverStatus status;
    // int32_t currentPosition = 0;

    RCLCPP_INFO(rclcpp::get_logger("Ti5RobotCRADriver"), "activateWithPositionMode(0x%x, 0x%08x)", can_id, currentPosition);

    // status = getPosition(can_id, currentPosition);
    // if (status != Ti5RobotCRADriverStatus::SUCCESS) {
    //     return status;
    // }

    return setTargetPosition(can_id, currentPosition);
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::activateWithVelocityMode(uint32_t can_id)
{
    Ti5RobotCRADriverStatus status;
    int32_t currentVelocity = 0;

    RCLCPP_INFO(rclcpp::get_logger("Ti5RobotCRADriver"), "activateWithVelocityMode(0x%x)", can_id);

    status = getVelocity(can_id, currentVelocity);
    if (status != Ti5RobotCRADriverStatus::SUCCESS) {
        return status;
    }

    return setTargetVelocity(can_id, currentVelocity);
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::activateWithCurrentMode(uint32_t can_id)
{
    Ti5RobotCRADriverStatus status;
    int32_t currentCurrent = 0;

    RCLCPP_INFO(rclcpp::get_logger("Ti5RobotCRADriver"), "activateWithCurrentMode(0x%x)", can_id);

    status = getCurrent(can_id, currentCurrent);
    if (status != Ti5RobotCRADriverStatus::SUCCESS) {
        return status;
    }

    return setTargetCurrent(can_id, currentCurrent);
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::deactivate(uint32_t can_id)
{
    RCLCPP_INFO(rclcpp::get_logger("Ti5RobotCRADriver"), "deactivate(0x%x)", can_id);    
    // no can message return
    return sendCanData(can_id, MotorCommand::MOTOR_HALT);
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::setTargetPosition(uint32_t can_id, int32_t value)
{
    // RCLCPP_INFO(rclcpp::get_logger("Ti5RobotCRADriver"), "setTargetPosition(0x%x, 0x%08x)", can_id, value);
    // no can message return
    return sendCanData(can_id, MotorCommand::SET_POSITION_MODE, value);
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::setTargetVelocity(uint32_t can_id, int32_t value)
{
    RCLCPP_INFO(rclcpp::get_logger("Ti5RobotCRADriver"), "setTargetVelocity(0x%x, 0x%08x)", can_id, value);
    // no can message return
    return sendCanData(can_id, MotorCommand::SET_VELOCITY_MODE, value);
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::setTargetCurrent(uint32_t can_id, int32_t value)
{
    RCLCPP_INFO(rclcpp::get_logger("Ti5RobotCRADriver"), "setTargetCurrent(0x%x, 0x%08x)", can_id, value);
    // no can message return
    return sendCanData(can_id, MotorCommand::SET_CURRENT_MODE, value);
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::getPosition(uint32_t can_id, int32_t& value)
{
  Ti5RobotCRADriverStatus status;

  // RCLCPP_INFO(rclcpp::get_logger("Ti5RobotCRADriver"), "getPosition(0x%x, 0x%08x)", can_id, value);
  status = sendRecvCanData(can_id, MotorCommand::GET_POSITION, value);
  // RCLCPP_INFO(rclcpp::get_logger("Ti5RobotCRADriver"), "getPosition(0x%x)--->%08x", can_id, value);

  return status;
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::getVelocity(uint32_t can_id, int32_t& value)
{
    return sendRecvCanData(can_id, MotorCommand::GET_VELOCITY, value);
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::getCurrent(uint32_t can_id, int32_t& value)
{
    return sendRecvCanData(can_id, MotorCommand::GET_CURRENT, value);
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::getPositionP(uint32_t can_id, int32_t& value)
{
  return sendRecvCanData(can_id, MotorCommand::GET_POSITION_P, value);
  // int32_t tmp;
  // Ti5RobotCRADriverStatus status = sendRecvCanData(can_id, MotorCommand::GET_POSITION_P, tmp);

  // if (status == Ti5RobotCRADriverStatus::SUCCESS)
  //   value = tmp & 0x7FF;

  // return status;
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::getPositionI(uint32_t can_id, int32_t& value)
{
  return sendRecvCanData(can_id, MotorCommand::GET_POSITION_I, value);
  // int32_t tmp;
  // Ti5RobotCRADriverStatus status = sendRecvCanData(can_id, MotorCommand::GET_POSITION_I, tmp);

  // if (status == Ti5RobotCRADriverStatus::SUCCESS)
  //   value = tmp & 0x7FF;

  // return status;
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::getPositionD(uint32_t can_id, int32_t& value)
{
  return sendRecvCanData(can_id, MotorCommand::GET_POSITION_D, value);
  // int32_t tmp;
  // Ti5RobotCRADriverStatus status = sendRecvCanData(can_id, MotorCommand::GET_POSITION_D, tmp);

  // if (status == Ti5RobotCRADriverStatus::SUCCESS)
  //   value = tmp & 0x7FF;

  // return status;
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::getVelocityP(uint32_t can_id, int32_t& value)
{
  return sendRecvCanData(can_id, MotorCommand::GET_VELOCITY_P, value);
  // int32_t tmp;
  // Ti5RobotCRADriverStatus status = sendRecvCanData(can_id, MotorCommand::GET_VELOCITY_P, tmp);

  // if (status == Ti5RobotCRADriverStatus::SUCCESS)
  //   value = tmp & 0x7FF;

  // return status;
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::getVelocityI(uint32_t can_id, int32_t& value)
{
  return sendRecvCanData(can_id, MotorCommand::GET_VELOCITY_I, value);
  // int32_t tmp;
  // Ti5RobotCRADriverStatus status = sendRecvCanData(can_id, MotorCommand::GET_VELOCITY_I, tmp);

  // if (status == Ti5RobotCRADriverStatus::SUCCESS)
  //   value = tmp & 0x7FF;

  // return status;
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::getVelocityD(uint32_t can_id, int32_t& value)
{
  return sendRecvCanData(can_id, MotorCommand::GET_VELOCITY_D, value);
  // int32_t tmp;
  // Ti5RobotCRADriverStatus status = sendRecvCanData(can_id, MotorCommand::GET_VELOCITY_D, tmp);

  // if (status == Ti5RobotCRADriverStatus::SUCCESS)
  //   value = tmp & 0x7FF;

  // return status;
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::getCurrentP(uint32_t can_id, int32_t& value)
{
  return sendRecvCanData(can_id, MotorCommand::GET_CURRENT_P, value);
  // int32_t tmp;
  // Ti5RobotCRADriverStatus status = sendRecvCanData(can_id, MotorCommand::GET_CURRENT_P, tmp);

  // if (status == Ti5RobotCRADriverStatus::SUCCESS)
  //   value = tmp & 0x7FF;

  // return status;
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::getCurrentI(uint32_t can_id, int32_t& value)
{
  return sendRecvCanData(can_id, MotorCommand::GET_CURRENT_I, value);
  // int32_t tmp;
  // Ti5RobotCRADriverStatus status = sendRecvCanData(can_id, MotorCommand::GET_CURRENT_I, tmp);

  // if (status == Ti5RobotCRADriverStatus::SUCCESS)
  //   value = tmp & 0x7FF;

  // return status;
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::getCurrentD(uint32_t can_id, int32_t& value)
{
  return sendRecvCanData(can_id, MotorCommand::GET_CURRENT_D, value);
  // int32_t tmp;
  // Ti5RobotCRADriverStatus status = sendRecvCanData(can_id, MotorCommand::GET_CURRENT_D, tmp);

  // if (status == Ti5RobotCRADriverStatus::SUCCESS)
  //   value = tmp & 0x7FF;

  // return status;
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::getMotorTemperature(uint32_t can_id, int32_t& value)
{
    return sendRecvCanData(can_id, MotorCommand::GET_MOTOR_TEMP, value);
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::getBoardTemperature(uint32_t can_id, int32_t& value)
{
    return sendRecvCanData(can_id, MotorCommand::GET_BOARD_TEMP, value);
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::sendRecvCanData(uint32_t can_id, uint8_t cmd, int32_t& ret)
{
  Ti5RobotCRADriverStatus status;

  status = sendCanData(can_id, cmd, ret);
  if (status != Ti5RobotCRADriverStatus::SUCCESS) {
      return status;
  }

  return recvCanData(can_id, cmd, ret);
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::sendCanData(uint32_t can_id, uint8_t cmd, int32_t value)
{
    int dlen = 0;
    uint8_t data[CAN_MAX_DATA_LEN] = {0};
    CanId send_id = CanId(can_id, 0, FrameType::DATA, StandardFrame);

    data[0] = cmd;

    if (cmd == MotorCommand::MOTOR_HALT || 
        cmd == MotorCommand::CLEAR_ERRORS ||
        cmd == MotorCommand::GET_POSITION ||
        cmd == MotorCommand::GET_VELOCITY ||
        cmd == MotorCommand::GET_CURRENT ||
        cmd == MotorCommand::GET_POSITION_P ||
        cmd == MotorCommand::GET_POSITION_I ||
        cmd == MotorCommand::GET_POSITION_D ||
        cmd == MotorCommand::GET_VELOCITY_P ||
        cmd == MotorCommand::GET_VELOCITY_I ||
        cmd == MotorCommand::GET_VELOCITY_D ||
        cmd == MotorCommand::GET_CURRENT_P ||
        cmd == MotorCommand::GET_CURRENT_I ||
        cmd == MotorCommand::GET_CURRENT_D ||
        cmd == MotorCommand::GET_MOTOR_TEMP ||
        cmd == MotorCommand::GET_BOARD_TEMP) 
    {
      dlen = 1;
    } else {
      data[1] = static_cast<uint8_t>(value & 0xFF);
      data[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
      data[3] = static_cast<uint8_t>((value >> 16) & 0xFF);
      data[4] = static_cast<uint8_t>((value >> 24) & 0xFF);
      dlen = 5;
    }

    try {
        can_driver_->send(data, dlen, send_id, sender_timeout_ns_);
    } catch (const std::exception & ex) {
        RCLCPP_ERROR(rclcpp::get_logger("Ti5RobotCRADriver"), "Error sending CAN message, interface: %s - %s", interface_.c_str(), ex.what());
        return Ti5RobotCRADriverStatus::CAN_SENDING_ERROR;
    }

    return Ti5RobotCRADriverStatus::SUCCESS;
}

Ti5RobotCRADriverStatus Ti5RobotCRADriver::recvCanData(uint32_t can_id, uint8_t cmd, int32_t& ret)
{
    uint8_t data[CAN_MAX_DATA_LEN];
    CanId receive_id{};
    
    ret = 0;

    try {
        receive_id = can_driver_->receive((void *)data, receiver_timeout_ns_);
    } catch (const std::exception & ex) {
        RCLCPP_ERROR(rclcpp::get_logger("Ti5RobotCRADriver"), "Error receiving CAN message, interface: %s -- %s", 
                        interface_.c_str(), ex.what());
        return Ti5RobotCRADriverStatus::CAN_RECVING_ERROR;
    }

    if (can_id != receive_id.get()) {
        RCLCPP_ERROR(rclcpp::get_logger("Ti5RobotCRADriver"), "Unmatched can_id, can_id: 0x%x -- received_id: 0x%x", can_id, receive_id.get());
        return Ti5RobotCRADriverStatus::CAN_UNMATCHED_ID;
    }

    if (data[0] != cmd) {
        RCLCPP_ERROR(rclcpp::get_logger("Ti5RobotCRADriver"), "Unmatched command, cmd: 0x%x -- received cmd: 0x%x", cmd, data[0]);
        return Ti5RobotCRADriverStatus::CAN_UNMATCHED_CMD;
    }

    ret = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);

    return Ti5RobotCRADriverStatus::SUCCESS;
}
