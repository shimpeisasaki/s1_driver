#ifndef S1_DRIVER_HPP_
#define S1_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "s1_driver/rust_bridge.hpp"

#include <memory>
#include <string>
#include <vector>
#include <mutex>

namespace s1_driver
{

class S1Driver : public rclcpp::Node
{
public:
  explicit S1Driver();
  ~S1Driver();

private:
  // Callback functions
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void livoxImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void timerCallback();
  
  // CAN communication functions
  bool initializeCAN();
  void shutdownCAN();
  bool sendMovementCommand(double vx, double vy, double vz);
  bool readSensorData();
  
  // Data processing functions
  void updateOdometry();
  void publishTF();
  void publishSensorData();
  
  // ROS2 publishers and subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr livox_imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  // Livox IMU data
  double livox_gyro_z_ = 0.0;
  bool has_livox_imu_ = false;
  std::mutex livox_imu_mutex_;

  // Gyro bias calibration
  double gyro_z_bias_ = 0.0;
  double gyro_z_bias_sum_ = 0.0;
  int gyro_calibration_count_ = 0;
  bool gyro_calibrated_ = false;
  static constexpr int GYRO_CALIBRATION_SAMPLES = 100;  // ~1 second at 100Hz
  
  // TF broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // Timer for periodic updates
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Robot state variables
  struct RobotState {
    double x = 0.0, y = 0.0, theta = 0.0;  // Position and orientation
    double vx = 0.0, vy = 0.0, vtheta = 0.0;  // Velocities (body frame)
    double wheel_positions[4] = {0};  // Wheel positions [rad]
    double wheel_velocities[4] = {0};  // Wheel velocities [rad/s]
    double imu_accel[3] = {0, 0, 9.81};  // Linear acceleration [m/s²]
    double imu_gyro[3] = {0};  // Angular velocity [rad/s]

    // Motion controller data
    double controller_vx = 0.0, controller_vy = 0.0, controller_vz = 0.0;

    // Attitude data
    double roll = 0.0, pitch = 0.0, yaw = 0.0;

    // Data availability flags
    bool has_wheel_data = false;
    bool has_imu_data = false;
    bool has_controller_velocity = false;
    bool has_attitude_data = false;

    rclcpp::Time timestamp;
  } robot_state_;
  
  // Parameters
  std::string can_interface_;
  std::string base_frame_;
  std::string odom_frame_;
  double control_frequency_;
  double cmd_vel_timeout_;
  bool publish_tf_;
  
  // Safety and control variables
  rclcpp::Time last_cmd_vel_time_;
  bool emergency_stop_;
  bool can_initialized_;
  
  // Rust bridge for CAN communication
  std::unique_ptr<RustBridge> rust_bridge_;
  
  // Constants
  static constexpr double WHEEL_RADIUS = 0.05;        // meters
  static constexpr double WHEEL_BASE_WIDTH = 0.2;     // meters  
  static constexpr double WHEEL_BASE_LENGTH = 0.2;    // meters

  // Velocity limits
  double max_linear_velocity_fwd_;
  double max_linear_velocity_bwd_;
  double max_linear_velocity_lat_;
  double max_angular_velocity_;
};

} // namespace s1_driver

#endif // S1_DRIVER_HPP_
