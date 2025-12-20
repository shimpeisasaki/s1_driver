#include "s1_driver/rust_bridge.hpp"
#include "s1_driver/rust_interface.h"
#include <iostream>

namespace s1_driver
{

RustBridge::RustBridge(const std::string& can_interface)
  : handle_id_(-1)
  , can_interface_(can_interface)
  , is_initialized_(false)
{
  handle_id_ = rust_bridge_create(can_interface.c_str());
}

RustBridge::~RustBridge()
{
  if (handle_id_ >= 0) {
    rust_bridge_destroy(handle_id_);
    handle_id_ = -1;
  }
}

bool RustBridge::initialize()
{
  if (!isValidHandle()) {
    return false;
  }
  
  is_initialized_ = rust_bridge_initialize(handle_id_, can_interface_.c_str()) != 0;
  return is_initialized_;
}

void RustBridge::setGains(double x, double y, double z)
{
  if (!isValidHandle()) {
    return;
  }
  rust_bridge_set_gains(handle_id_, static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
}

bool RustBridge::move(double vx, double vy, double vz)
{
  if (!isValidHandle() || !is_initialized_) {
    return false;
  }
  
  MovementParams params;
  params.vx = static_cast<float>(vx);
  params.vy = static_cast<float>(vy);
  params.vz = static_cast<float>(vz);
  
  return rust_bridge_send_movement(handle_id_, params) != 0;
}

bool RustBridge::stop()
{
  if (!isValidHandle() || !is_initialized_) {
    return false;
  }
  
  return rust_bridge_stop(handle_id_) != 0;
}

bool RustBridge::readSensors(SensorData& data)
{
  if (!isValidHandle() || !is_initialized_) {
    return false;
  }

  return rust_bridge_read_sensor_data(handle_id_) != 0;
}

bool RustBridge::getEscData(EscData& data)
{
  if (!isValidHandle() || !is_initialized_) {
    return false;
  }

  return rust_bridge_get_esc_data(handle_id_, &data) != 0;
}

bool RustBridge::getImuData(ImuData& data)
{
  if (!isValidHandle() || !is_initialized_) {
    return false;
  }

  return rust_bridge_get_imu_data_new(handle_id_, &data) != 0;
}

bool RustBridge::getVelocityData(VelocityData& data)
{
  if (!isValidHandle() || !is_initialized_) {
    return false;
  }

  return rust_bridge_get_velocity_data(handle_id_, &data) != 0;
}

bool RustBridge::getPositionData(PositionData& data)
{
  if (!isValidHandle() || !is_initialized_) {
    return false;
  }

  return rust_bridge_get_position_data(handle_id_, &data) != 0;
}

bool RustBridge::getAttitudeData(AttitudeData& data)
{
  if (!isValidHandle() || !is_initialized_) {
    return false;
  }

  return rust_bridge_get_attitude_data(handle_id_, &data) != 0;
}

bool RustBridge::setLed(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
  if (!isValidHandle() || !is_initialized_) {
    return false;
  }
  
  LedColor color;
  color.red = r;
  color.green = g;
  color.blue = b;
  
  return rust_bridge_send_led_color(handle_id_, color) != 0;
}

bool RustBridge::isConnected() const
{
  if (!isValidHandle()) {
    return false;
  }
  
  // For now, assume connected if initialized
  // The actual implementation would check the robot status
  return is_initialized_;
}

std::string RustBridge::getLastError() const
{
  if (!isValidHandle()) {
    return "Invalid handle";
  }
  
  const char* error = rust_bridge_get_last_error(handle_id_);
  if (error) {
    std::string result(error);
    rust_bridge_free_string(const_cast<char*>(error));
    return result;
  }
  
  return "";
}

bool RustBridge::isValidHandle() const
{
  return handle_id_ >= 0;
}

}  // namespace s1_driver
