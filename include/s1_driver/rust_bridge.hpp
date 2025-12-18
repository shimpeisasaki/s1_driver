#ifndef S1_DRIVER__RUST_BRIDGE_HPP_
#define S1_DRIVER__RUST_BRIDGE_HPP_

#include "s1_driver/rust_interface.h"
#include <string>
#include <memory>

namespace s1_driver
{

/**
 * @brief C++ wrapper for Rust RoboMaster interface
 * 
 * This class provides a RAII-style C++ interface to the Rust-based RoboMaster control library.
 * It handles resource management, error handling, and type conversions between C++ and C interfaces.
 */
class RustBridge
{
public:
  /**
   * @brief Construct a new Rust Bridge object
   * @param can_interface CAN interface name (e.g., "can0")
   */
  explicit RustBridge(const std::string& can_interface);
  
  /**
   * @brief Destroy the Rust Bridge object
   * Automatically cleans up Rust resources
   */
  ~RustBridge();
  
  // Delete copy constructor and assignment operator
  RustBridge(const RustBridge&) = delete;
  RustBridge& operator=(const RustBridge&) = delete;
  
  /**
   * @brief Initialize the robot connection
   * @return true if initialization successful, false otherwise
   */
  bool initialize();
  
  /**
   * @brief Send movement command to robot
   * @param vx Forward/backward velocity (-1.0 to 1.0)
   * @param vy Left/right strafe velocity (-1.0 to 1.0)  
   * @param vz Rotation velocity (-1.0 to 1.0)
   * @return true if command sent successfully, false otherwise
   */
  bool move(double vx, double vy, double vz);
  
  /**
   * @brief Send boot sequence to initialize robot mode
   * @return true if command sent successfully, false otherwise
   */
  bool sendBootSequence();

  /**
   * @brief Stop robot movement
   * @return true if stop command sent successfully, false otherwise
   */
  bool stop();
  
  /**
   * @brief Read sensor data from robot (triggers CAN receive)
   * @param data Reference to SensorData structure to fill
   * @return true if sensor data read successfully, false otherwise
   */
  bool readSensors(SensorData& data);

  /**
   * @brief Get ESC (wheel motor) data
   * @param data Reference to EscData structure to fill
   * @return true if data retrieved successfully, false otherwise
   */
  bool getEscData(EscData& data);

  /**
   * @brief Get IMU data
   * @param data Reference to ImuData structure to fill
   * @return true if data retrieved successfully, false otherwise
   */
  bool getImuData(ImuData& data);

  /**
   * @brief Get velocity data from motion controller
   * @param data Reference to VelocityData structure to fill
   * @return true if data retrieved successfully, false otherwise
   */
  bool getVelocityData(VelocityData& data);

  /**
   * @brief Get position data from motion controller
   * @param data Reference to PositionData structure to fill
   * @return true if data retrieved successfully, false otherwise
   */
  bool getPositionData(PositionData& data);

  /**
   * @brief Get attitude data
   * @param data Reference to AttitudeData structure to fill
   * @return true if data retrieved successfully, false otherwise
   */
  bool getAttitudeData(AttitudeData& data);

  /**
   * @brief Set LED color
   * @param r Red component (0-255)
   * @param g Green component (0-255)
   * @param b Blue component (0-255)
   * @param brightness Overall brightness (0-255)
   * @return true if LED command sent successfully, false otherwise
   */
  bool setLed(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness = 255);
  
  /**
   * @brief Check if robot is connected
   * @return true if connected, false otherwise
   */
  bool isConnected() const;
  
  /**
   * @brief Get last error message
   * @return Error message string, empty if no error
   */
  std::string getLastError() const;

private:
  int32_t handle_id_;               ///< Handle ID from Rust bridge
  std::string can_interface_;       ///< CAN interface name
  bool is_initialized_;             ///< Initialization status flag
  
  /**
   * @brief Check if the internal handle is valid
   * @return true if handle is valid, false otherwise
   */
  bool isValidHandle() const;
};

}  // namespace s1_driver

#endif  // S1_DRIVER__RUST_BRIDGE_HPP_
