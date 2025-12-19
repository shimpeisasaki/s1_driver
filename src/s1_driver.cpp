#include "s1_driver/s1_driver.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

namespace s1_driver
{

S1Driver::S1Driver() : Node("s1_driver")
{
  // Declare parameters
  this->declare_parameter<std::string>("can_interface", "can2");
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<double>("control_frequency", 50.0);
  this->declare_parameter<double>("cmd_vel_timeout", 1.0);
  this->declare_parameter<bool>("publish_tf", true);
  
  // Declare velocity limit parameters
  this->declare_parameter<double>("max_vel_fwd", 3.5);
  this->declare_parameter<double>("max_vel_bwd", 2.5);
  this->declare_parameter<double>("max_vel_lat", 2.8);
  this->declare_parameter<double>("max_vel_ang", 3.0);
  
  // Get parameters
  can_interface_ = this->get_parameter("can_interface").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  odom_frame_ = this->get_parameter("odom_frame").as_string();
  control_frequency_ = this->get_parameter("control_frequency").as_double();
  cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();
  publish_tf_ = this->get_parameter("publish_tf").as_bool();
  
  // Get velocity limit parameters
  max_linear_velocity_fwd_ = this->get_parameter("max_vel_fwd").as_double();
  max_linear_velocity_bwd_ = this->get_parameter("max_vel_bwd").as_double();
  max_linear_velocity_lat_ = this->get_parameter("max_vel_lat").as_double();
  max_angular_velocity_ = this->get_parameter("max_vel_ang").as_double();
  
  RCLCPP_INFO(this->get_logger(), "S1 Driver starting with CAN interface: %s", can_interface_.c_str());
  
  // Initialize state
  robot_state_ = {};
  robot_state_.timestamp = this->now();
  emergency_stop_ = false;
  can_initialized_ = false;
  last_cmd_vel_time_ = this->now();
  
  // Initialize Rust bridge
  rust_bridge_ = std::make_unique<RustBridge>(can_interface_);
  
  // Create publishers
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  
  // Create subscribers
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&S1Driver::cmdVelCallback, this, std::placeholders::_1));

  // Subscribe to Livox IMU for yaw calculation
  livox_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/livox/imu", 10,
    std::bind(&S1Driver::livoxImuCallback, this, std::placeholders::_1));

  // Create TF broadcaster
  if (publish_tf_) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }
  
  // Initialize CAN communication
  if (!initializeCAN()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN communication");
    return;
  }
  
  // Create timer for periodic updates
  auto timer_period = std::chrono::duration<double>(1.0 / control_frequency_);
  timer_ = this->create_wall_timer(
    timer_period,
    std::bind(&S1Driver::timerCallback, this));
  
  RCLCPP_INFO(this->get_logger(), "S1 Driver initialized successfully");
}

S1Driver::~S1Driver()
{
  shutdownCAN();
}

void S1Driver::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (emergency_stop_ || !can_initialized_) {
    return;
  }
  
  // Update last command time
  last_cmd_vel_time_ = this->now();
  
  // Extract velocities
  double vx = msg->linear.x;
  if (vx >= 0) {
    vx = std::clamp(vx, 0.0, max_linear_velocity_fwd_);
  } else {
    vx = std::clamp(vx, -max_linear_velocity_bwd_, 0.0);
  }
  
  double vy = std::clamp(msg->linear.y, -max_linear_velocity_lat_, max_linear_velocity_lat_);
  double vz = std::clamp(msg->angular.z, -max_angular_velocity_, max_angular_velocity_);
  
  // Send movement command
  if (!sendMovementCommand(vx, vy, vz)) {
    RCLCPP_WARN(this->get_logger(), "Failed to send movement command");
  }
}

void S1Driver::livoxImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(livox_imu_mutex_);

  double raw_gyro_z = msg->angular_velocity.z;

  // Calibration phase: collect samples to compute bias
  if (!gyro_calibrated_) {
    gyro_z_bias_sum_ += raw_gyro_z;
    gyro_calibration_count_++;

    if (gyro_calibration_count_ >= GYRO_CALIBRATION_SAMPLES) {
      gyro_z_bias_ = gyro_z_bias_sum_ / gyro_calibration_count_;
      gyro_calibrated_ = true;
      RCLCPP_INFO(this->get_logger(),
        "Gyro Z bias calibrated: %.6f rad/s (keep robot stationary during startup)",
        gyro_z_bias_);
    }
    return;  // Don't use IMU data until calibrated
  }

  // Apply bias correction
  livox_gyro_z_ = raw_gyro_z - gyro_z_bias_;
  has_livox_imu_ = true;
}

void S1Driver::timerCallback()
{
  if (!can_initialized_) {
    return;
  }
  
  // Check for command timeout
  auto current_time = this->now();
  auto time_since_last_cmd = (current_time - last_cmd_vel_time_).seconds();
  
  if (time_since_last_cmd > cmd_vel_timeout_) {
    // Send stop command
    sendMovementCommand(0.0, 0.0, 0.0);
  }
  
  // Read sensor data from robot
  if (readSensorData()) {
    updateOdometry();
    publishSensorData();
    
    if (publish_tf_) {
      publishTF();
    }
  }
}

bool S1Driver::initializeCAN()
{
  RCLCPP_INFO(this->get_logger(), "Initializing CAN communication...");
  
  if (!rust_bridge_) {
    RCLCPP_ERROR(this->get_logger(), "Rust bridge not initialized");
    return false;
  }
  
  if (!rust_bridge_->initialize()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize Rust bridge: %s", 
                 rust_bridge_->getLastError().c_str());
    return false;
  }
  
  can_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "CAN communication initialized successfully");
  return true;
}

void S1Driver::shutdownCAN()
{
  if (can_initialized_) {
    RCLCPP_INFO(this->get_logger(), "Shutting down CAN communication");
    
    // Stop the robot before shutdown
    if (rust_bridge_) {
      rust_bridge_->stop();
    }
    
    can_initialized_ = false;
  }
}

bool S1Driver::sendMovementCommand(double vx, double vy, double vz)
{
  if (!rust_bridge_ || !can_initialized_) {
    return false;
  }
  
  RCLCPP_DEBUG(this->get_logger(), 
    "Sending movement command: vx=%.2f, vy=%.2f, vz=%.2f", vx, vy, vz);
  
  // Normalize velocities to -1.0 to 1.0 range expected by robomaster-rust
  /*
  double norm_vx;
  if (vx >= 0) {
    norm_vx = vx / max_linear_velocity_fwd_;
  } else {
    norm_vx = vx / max_linear_velocity_bwd_;
  }
  
  double norm_vy = vy / max_linear_velocity_lat_;
  double norm_vz = vz / max_angular_velocity_;
  */
  
  if (!rust_bridge_->move(vx, vy, vz)) {
    RCLCPP_WARN(this->get_logger(), "Failed to send movement command: %s",
                rust_bridge_->getLastError().c_str());
    return false;
  }
  
  return true;
}

bool S1Driver::readSensorData()
{
  if (!rust_bridge_ || !can_initialized_) {
    return false;
  }

  // Trigger sensor data receive from CAN
  SensorData sensor_data;
  if (!rust_bridge_->readSensors(sensor_data)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Failed to read sensor data: %s",
                         rust_bridge_->getLastError().c_str());
    return false;
  }

  robot_state_.timestamp = this->now();

  // Get ESC (wheel) data
  EscData esc_data;
  if (rust_bridge_->getEscData(esc_data)) {
    if (esc_data.has_data) {
      for (int i = 0; i < 4; ++i) {
        robot_state_.wheel_velocities[i] = esc_data.speeds[i];
        robot_state_.wheel_positions[i] = esc_data.angles[i];
      }
      robot_state_.has_wheel_data = true;
      RCLCPP_DEBUG(this->get_logger(), "ESC: speeds=[%.2f, %.2f, %.2f, %.2f] rad/s",
        robot_state_.wheel_velocities[0], robot_state_.wheel_velocities[1],
        robot_state_.wheel_velocities[2], robot_state_.wheel_velocities[3]);
    }
  }

  // Get IMU data
  ImuData imu_data;
  if (rust_bridge_->getImuData(imu_data) && imu_data.has_data) {
    robot_state_.imu_accel[0] = imu_data.accel[0];
    robot_state_.imu_accel[1] = imu_data.accel[1];
    robot_state_.imu_accel[2] = imu_data.accel[2];
    robot_state_.imu_gyro[0] = imu_data.gyro[0];
    robot_state_.imu_gyro[1] = imu_data.gyro[1];
    robot_state_.imu_gyro[2] = imu_data.gyro[2];
    robot_state_.has_imu_data = true;
  }

  // Get velocity data from motion controller (if available)
  VelocityData vel_data;
  if (rust_bridge_->getVelocityData(vel_data) && vel_data.has_data) {
    robot_state_.controller_vx = vel_data.body[0];
    robot_state_.controller_vy = vel_data.body[1];
    robot_state_.controller_vz = vel_data.body[2];
    robot_state_.has_controller_velocity = true;
  }

  // Get attitude data
  AttitudeData attitude_data;
  if (rust_bridge_->getAttitudeData(attitude_data) && attitude_data.has_data) {
    robot_state_.roll = attitude_data.roll;
    robot_state_.pitch = attitude_data.pitch;
    robot_state_.yaw = attitude_data.yaw;
    robot_state_.has_attitude_data = true;
  }

  return true;
}

void S1Driver::updateOdometry()
{
  auto current_time = robot_state_.timestamp;
  static auto last_time = current_time;

  double dt = (current_time - last_time).seconds();
  if (dt <= 0.0 || dt > 1.0) {
    last_time = current_time;
    return;
  }

  // Calculate velocities from wheel encoders using Mecanum kinematics
  // Wheel order: [0]=FL, [1]=FR, [2]=RL, [3]=RR
  // For Mecanum wheels:
  //   vx = (w0 + w1 + w2 + w3) * R / 4
  //   vy = (-w0 + w1 + w2 - w3) * R / 4
  //   omega = (-w0 + w1 - w2 + w3) * R / (4 * (L + W))
  // where R = wheel radius, L = half wheelbase length, W = half wheelbase width

  if (robot_state_.has_wheel_data) {
    double w0 = robot_state_.wheel_velocities[0];  // Front-left
    double w1 = robot_state_.wheel_velocities[1];  // Front-right
    double w2 = robot_state_.wheel_velocities[2];  // Rear-left
    double w3 = robot_state_.wheel_velocities[3];  // Rear-right

    // Mecanum forward kinematics (body frame velocities)
    double vx_body = (w0 + w1 + w2 + w3) * WHEEL_RADIUS / 4.0;
    double vy_body = (-w0 + w1 + w2 - w3) * WHEEL_RADIUS / 4.0;
    double omega = (-w0 + w1 - w2 + w3) * WHEEL_RADIUS /
                   (4.0 * (WHEEL_BASE_LENGTH / 2.0 + WHEEL_BASE_WIDTH / 2.0));

    // Store body frame velocities
    robot_state_.vx = vx_body;
    robot_state_.vy = vy_body;
    robot_state_.vtheta = omega;
  } else if (robot_state_.has_controller_velocity) {
    // Use velocity from motion controller if wheel data not available
    // Sanity check: velocities should be within reasonable bounds (< 10 m/s)
    double vx = robot_state_.controller_vx;
    double vy = robot_state_.controller_vy;
    double vz = robot_state_.controller_vz;  // Angular velocity from motion controller
    if (std::isfinite(vx) && std::isfinite(vy) &&
        std::abs(vx) < 10.0 && std::abs(vy) < 10.0) {
      robot_state_.vx = vx;
      robot_state_.vy = vy;
      // Use vz from velocity data as angular velocity (body[2] contains omega)
      if (std::isfinite(vz) && std::abs(vz) < 20.0) {
        robot_state_.vtheta = vz;
      } else if (robot_state_.has_imu_data) {
        // Fallback to IMU gyro if vz is invalid
        double vtheta = robot_state_.imu_gyro[2];
        if (std::isfinite(vtheta) && std::abs(vtheta) < 20.0) {
          robot_state_.vtheta = vtheta;
        }
      }
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Invalid velocity data: vx=%.3f, vy=%.3f (filtering out)", vx, vy);
      robot_state_.vx = 0.0;
      robot_state_.vy = 0.0;
      robot_state_.vtheta = 0.0;
    }
  }

  // Override angular velocity with Livox IMU if available (more accurate for yaw)
  {
    std::lock_guard<std::mutex> lock(livox_imu_mutex_);
    if (has_livox_imu_) {
      if (std::isfinite(livox_gyro_z_) && std::abs(livox_gyro_z_) < 20.0) {
        robot_state_.vtheta = livox_gyro_z_;
      }
    }
  }

  // Transform body velocities to global frame and integrate
  double cos_theta = std::cos(robot_state_.theta);
  double sin_theta = std::sin(robot_state_.theta);

  double dx_global = (robot_state_.vx * cos_theta - robot_state_.vy * sin_theta) * dt;
  double dy_global = (robot_state_.vx * sin_theta + robot_state_.vy * cos_theta) * dt;
  double dtheta = robot_state_.vtheta * dt;

  // Update pose
  robot_state_.x += dx_global;
  robot_state_.y += dy_global;
  robot_state_.theta += dtheta;

  // Normalize theta to [-PI, PI]
  while (robot_state_.theta > M_PI) robot_state_.theta -= 2.0 * M_PI;
  while (robot_state_.theta < -M_PI) robot_state_.theta += 2.0 * M_PI;

  // Fallback: If Livox IMU not available and attitude data is available, use yaw directly
  if (!has_livox_imu_ && robot_state_.has_attitude_data) {
    double yaw = robot_state_.yaw;
    // Sanity check: yaw should be within [-2*PI, 2*PI]
    if (std::isfinite(yaw) && std::abs(yaw) < 2.0 * M_PI) {
      robot_state_.theta = yaw;
    }
  }

  last_time = current_time;
}

void S1Driver::publishSensorData()
{
  auto current_time = robot_state_.timestamp;

  // Publish odometry
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  // Position
  odom_msg.pose.pose.position.x = robot_state_.x;
  odom_msg.pose.pose.position.y = robot_state_.y;
  odom_msg.pose.pose.position.z = 0.0;

  // Orientation (use roll/pitch if available)
  tf2::Quaternion q;
  if (robot_state_.has_attitude_data) {
    q.setRPY(robot_state_.roll, robot_state_.pitch, robot_state_.theta);
  } else {
    q.setRPY(0, 0, robot_state_.theta);
  }
  odom_msg.pose.pose.orientation = tf2::toMsg(q);

  // Velocity (body frame)
  odom_msg.twist.twist.linear.x = robot_state_.vx;
  odom_msg.twist.twist.linear.y = robot_state_.vy;
  odom_msg.twist.twist.angular.z = robot_state_.vtheta;

  // Set covariance matrices (diagonal, values depend on sensor quality)
  // Pose covariance: [x, y, z, roll, pitch, yaw]
  odom_msg.pose.covariance[0] = 0.01;   // x variance
  odom_msg.pose.covariance[7] = 0.01;   // y variance
  odom_msg.pose.covariance[14] = 1e6;   // z variance (large, not measured)
  odom_msg.pose.covariance[21] = 1e6;   // roll variance
  odom_msg.pose.covariance[28] = 1e6;   // pitch variance
  odom_msg.pose.covariance[35] = 0.03;  // yaw variance

  // Twist covariance
  odom_msg.twist.covariance[0] = 0.01;   // vx variance
  odom_msg.twist.covariance[7] = 0.01;   // vy variance
  odom_msg.twist.covariance[14] = 1e6;   // vz variance
  odom_msg.twist.covariance[21] = 1e6;   // angular x variance
  odom_msg.twist.covariance[28] = 1e6;   // angular y variance
  odom_msg.twist.covariance[35] = 0.03;  // angular z variance

  odom_pub_->publish(odom_msg);

  // Publish IMU data
  auto imu_msg = sensor_msgs::msg::Imu();
  imu_msg.header.stamp = current_time;
  imu_msg.header.frame_id = base_frame_;

  // Orientation from attitude if available
  if (robot_state_.has_attitude_data) {
    tf2::Quaternion imu_q;
    imu_q.setRPY(robot_state_.roll, robot_state_.pitch, robot_state_.yaw);
    imu_msg.orientation = tf2::toMsg(imu_q);
    imu_msg.orientation_covariance[0] = 0.01;
    imu_msg.orientation_covariance[4] = 0.01;
    imu_msg.orientation_covariance[8] = 0.01;
  } else {
    // Orientation not available
    imu_msg.orientation_covariance[0] = -1;
  }

  imu_msg.angular_velocity.x = robot_state_.imu_gyro[0];
  imu_msg.angular_velocity.y = robot_state_.imu_gyro[1];
  imu_msg.angular_velocity.z = robot_state_.imu_gyro[2];
  imu_msg.angular_velocity_covariance[0] = 0.001;
  imu_msg.angular_velocity_covariance[4] = 0.001;
  imu_msg.angular_velocity_covariance[8] = 0.001;

  imu_msg.linear_acceleration.x = robot_state_.imu_accel[0];
  imu_msg.linear_acceleration.y = robot_state_.imu_accel[1];
  imu_msg.linear_acceleration.z = robot_state_.imu_accel[2];
  imu_msg.linear_acceleration_covariance[0] = 0.01;
  imu_msg.linear_acceleration_covariance[4] = 0.01;
  imu_msg.linear_acceleration_covariance[8] = 0.01;

  imu_pub_->publish(imu_msg);

  // Publish joint states
  auto joint_msg = sensor_msgs::msg::JointState();
  joint_msg.header.stamp = current_time;
  joint_msg.name = {"front_left_wheel_joint", "front_right_wheel_joint",
                    "rear_left_wheel_joint", "rear_right_wheel_joint"};

  joint_msg.position.resize(4);
  joint_msg.velocity.resize(4);

  for (int i = 0; i < 4; ++i) {
    joint_msg.position[i] = robot_state_.wheel_positions[i];
    joint_msg.velocity[i] = robot_state_.wheel_velocities[i];
  }

  joint_state_pub_->publish(joint_msg);
}

void S1Driver::publishTF()
{
  if (!tf_broadcaster_) return;
  
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = robot_state_.timestamp;
  transform.header.frame_id = odom_frame_;
  transform.child_frame_id = base_frame_;
  
  transform.transform.translation.x = robot_state_.x;
  transform.transform.translation.y = robot_state_.y;
  transform.transform.translation.z = 0.0;
  
  tf2::Quaternion q;
  q.setRPY(0, 0, robot_state_.theta);
  transform.transform.rotation = tf2::toMsg(q);
  
  tf_broadcaster_->sendTransform(transform);
}

} // namespace s1_driver
