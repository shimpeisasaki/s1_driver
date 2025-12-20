#ifndef RUST_INTERFACE_H_
#define RUST_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// C bool type definition
typedef uint8_t c_bool;

// Movement parameters structure
typedef struct {
    float vx;      // Forward/backward velocity (-1.0 to 1.0)
    float vy;      // Left/right strafe velocity (-1.0 to 1.0)
    float vz;      // Rotation velocity (-1.0 to 1.0)
} MovementParams;

// LED color structure
typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} LedColor;

// ESC (wheel motor) data structure
typedef struct {
    float speeds[4];     // Wheel speeds in rad/s [FL, FR, RL, RR]
    float angles[4];     // Wheel angles in radians
    c_bool has_data;     // Data valid flag
} EscData;

// IMU data structure
typedef struct {
    float accel[3];      // Acceleration in m/s² [x, y, z]
    float gyro[3];       // Angular velocity in rad/s [x, y, z]
    c_bool has_data;     // Data valid flag
} ImuData;

// Velocity data structure
typedef struct {
    float body[3];       // Body frame velocity [vx, vy, vz] in m/s
    c_bool has_data;     // Data valid flag
} VelocityData;

// Position data structure
typedef struct {
    float x;             // X position in meters
    float y;             // Y position in meters
    float z;             // Z position in meters
    c_bool has_data;     // Data valid flag
} PositionData;

// Attitude data structure
typedef struct {
    float yaw;           // Yaw angle in radians
    float pitch;         // Pitch angle in radians
    float roll;          // Roll angle in radians
    c_bool has_data;     // Data valid flag
} AttitudeData;

// Legacy sensor data structure (for compatibility)
typedef struct {
    // IMU data
    float accel_x, accel_y, accel_z;      // m/s^2
    float gyro_x, gyro_y, gyro_z;         // rad/s

    // Motor feedback (if available)
    float wheel_speeds[4];                // rad/s for each wheel

    // Battery status
    float battery_voltage;                // V
    float battery_current;                // A
    float battery_temperature;            // °C

    // System status
    c_bool is_connected;
    uint64_t timestamp_us;                // microseconds since epoch
} SensorData;

// Core robot control functions using handle IDs
int32_t rust_bridge_create(const char* interface);
c_bool rust_bridge_initialize(int32_t handle_id, const char* interface);
void rust_bridge_destroy(int32_t handle_id);

// Movement commands
c_bool rust_bridge_send_movement(int32_t handle_id, MovementParams params);
c_bool rust_bridge_stop(int32_t handle_id);
void rust_bridge_set_gains(int32_t handle_id, float x, float y, float z);

// LED control
c_bool rust_bridge_send_led_color(int32_t handle_id, LedColor color);

// Sensor data
c_bool rust_bridge_read_sensor_data(int32_t handle_id);
float rust_bridge_get_battery_level(int32_t handle_id);
c_bool rust_bridge_get_imu_data(int32_t handle_id,
                                float* acc_x, float* acc_y, float* acc_z,
                                float* gyro_x, float* gyro_y, float* gyro_z);

// New sensor data functions
c_bool rust_bridge_get_esc_data(int32_t handle_id, EscData* data);
c_bool rust_bridge_get_imu_data_new(int32_t handle_id, ImuData* data);
c_bool rust_bridge_get_velocity_data(int32_t handle_id, VelocityData* data);
c_bool rust_bridge_get_position_data(int32_t handle_id, PositionData* data);
c_bool rust_bridge_get_attitude_data(int32_t handle_id, AttitudeData* data);

// Error handling
const char* rust_bridge_get_last_error(int32_t handle_id);
void rust_bridge_free_string(char* s);

#ifdef __cplusplus
}
#endif

#endif  // RUST_INTERFACE_H_
