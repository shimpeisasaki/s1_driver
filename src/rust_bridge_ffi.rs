/// C interface bridge for robomaster-rust library
/// This module provides a C-compatible interface for integration with ROS2 C++ nodes

#[allow(non_camel_case_types)]

use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_float};
use std::sync::{Arc, Mutex};
use tokio::runtime::Runtime;
use anyhow::Result;

use robomaster_rust::control::RoboMaster;
use robomaster_rust::sensor::{SensorState, SharedSensorState};
use robomaster_rust::command::{MovementParams as RustMovementParams, LedColor as RustLedColor};

// C bool type definition
type c_bool = u8;

// C structure definitions matching the header
#[repr(C)]
pub struct MovementParams {
    pub vx: c_float,
    pub vy: c_float,
    pub vz: c_float,
}

impl From<MovementParams> for RustMovementParams {
    fn from(params: MovementParams) -> Self {
        RustMovementParams {
            vx: params.vx as f32,
            vy: params.vy as f32,
            vz: params.vz as f32,
        }
    }
}

#[repr(C)]
pub struct LedColor {
    pub red: u8,
    pub green: u8,
    pub blue: u8,
}

impl From<LedColor> for RustLedColor {
    fn from(color: LedColor) -> Self {
        RustLedColor {
            red: color.red,
            green: color.green,
            blue: color.blue,
        }
    }
}

/// ESC (wheel motor) data for FFI
#[repr(C)]
pub struct EscData {
    /// Wheel speeds in rad/s [front-left, front-right, rear-left, rear-right]
    pub speeds: [c_float; 4],
    /// Wheel angles in radians
    pub angles: [c_float; 4],
    /// Data valid flag
    pub has_data: c_bool,
}

/// IMU data for FFI
#[repr(C)]
pub struct ImuData {
    /// Acceleration in m/s² [x, y, z]
    pub accel: [c_float; 3],
    /// Angular velocity in rad/s [x, y, z]
    pub gyro: [c_float; 3],
    /// Data valid flag
    pub has_data: c_bool,
}

/// Velocity data for FFI
#[repr(C)]
pub struct VelocityData {
    /// Body frame velocity [vx, vy, vz] in m/s
    pub body: [c_float; 3],
    /// Data valid flag
    pub has_data: c_bool,
}

/// Position data for FFI
#[repr(C)]
pub struct PositionData {
    /// Position [x, y, z] in meters
    pub x: c_float,
    pub y: c_float,
    pub z: c_float,
    /// Data valid flag
    pub has_data: c_bool,
}

/// Attitude data for FFI
#[repr(C)]
pub struct AttitudeData {
    /// Yaw angle in radians
    pub yaw: c_float,
    /// Pitch angle in radians
    pub pitch: c_float,
    /// Roll angle in radians
    pub roll: c_float,
    /// Data valid flag
    pub has_data: c_bool,
}

/// Complete sensor data structure for FFI (legacy compatibility)
#[repr(C)]
pub struct SensorData {
    // IMU data
    pub accel_x: c_float,
    pub accel_y: c_float,
    pub accel_z: c_float,
    pub gyro_x: c_float,
    pub gyro_y: c_float,
    pub gyro_z: c_float,

    // Motor feedback (rad/s)
    pub wheel_speeds: [c_float; 4],

    // Battery status
    pub battery_voltage: c_float,
    pub battery_current: c_float,
    pub battery_temperature: c_float,

    // System status
    pub is_connected: c_bool,
    pub timestamp_us: u64,
}

// Bridge handle structure
pub struct RustBridgeHandle {
    pub robot: Option<Arc<Mutex<RoboMaster>>>,
    pub runtime: Runtime,
    pub sensor_state: Option<SharedSensorState>,
}

impl RustBridgeHandle {
    fn new() -> Result<Self> {
        let runtime = Runtime::new()?;
        Ok(RustBridgeHandle {
            robot: None,
            runtime,
            sensor_state: None,
        })
    }
}

// Global handles storage
static HANDLES: Mutex<Vec<Option<Arc<Mutex<RustBridgeHandle>>>>> = Mutex::new(Vec::new());

// C API functions
#[no_mangle]
pub extern "C" fn rust_bridge_create(interface: *const c_char) -> i32 {
    let _interface_str = if interface.is_null() {
        "can0".to_string()
    } else {
        match unsafe { CStr::from_ptr(interface) }.to_str() {
            Ok(s) => s.to_string(),
            Err(_) => {
                eprintln!("Failed to parse interface string");
                return -1;
            }
        }
    };

    let handle = match RustBridgeHandle::new() {
        Ok(h) => Arc::new(Mutex::new(h)),
        Err(e) => {
            eprintln!("Failed to create Rust bridge: {}", e);
            return -1;
        }
    };

    let mut handles = match HANDLES.lock() {
        Ok(h) => h,
        Err(_) => {
            eprintln!("Failed to lock handles mutex");
            return -1;
        }
    };

    // Find empty slot or add new one
    for (i, slot) in handles.iter_mut().enumerate() {
        if slot.is_none() {
            *slot = Some(handle);
            return i as i32;
        }
    }

    // No empty slot found, add new one
    handles.push(Some(handle));
    (handles.len() - 1) as i32
}

#[no_mangle]
pub extern "C" fn rust_bridge_initialize(handle_id: i32, interface: *const c_char) -> c_bool {
    let interface_str = if interface.is_null() {
        "can0".to_string()
    } else {
        match unsafe { CStr::from_ptr(interface) }.to_str() {
            Ok(s) => s.to_string(),
            Err(_) => {
                eprintln!("Failed to parse interface string");
                return 0;
            }
        }
    };

    let handles = match HANDLES.lock() {
        Ok(h) => h,
        Err(_) => {
            eprintln!("Failed to lock handles mutex");
            return 0;
        }
    };

    let handle = match handles.get(handle_id as usize) {
        Some(Some(h)) => h.clone(),
        _ => {
            eprintln!("Invalid handle ID: {}", handle_id);
            return 0;
        }
    };

    let result = handle.lock().unwrap().runtime.block_on(async move {
        let mut robot = RoboMaster::new(&interface_str).await?;
        robot.initialize().await?;
        let sensor_state = robot.get_shared_sensor_state();
        Ok::<(RoboMaster, SharedSensorState), anyhow::Error>((robot, sensor_state))
    });

    match result {
        Ok((robot, sensor_state)) => {
            let mut bridge = handle.lock().unwrap();
            bridge.robot = Some(Arc::new(Mutex::new(robot)));
            bridge.sensor_state = Some(sensor_state);
            1
        }
        Err(e) => {
            eprintln!("Failed to initialize robot: {}", e);
            0
        }
    }
}

#[no_mangle]
pub extern "C" fn rust_bridge_send_movement(handle_id: i32, params: MovementParams) -> c_bool {
    let handles = match HANDLES.lock() {
        Ok(h) => h,
        Err(_) => {
            eprintln!("Failed to lock handles mutex");
            return 0;
        }
    };

    let handle = match handles.get(handle_id as usize) {
        Some(Some(h)) => h.clone(),
        _ => {
            eprintln!("Invalid handle ID: {}", handle_id);
            return 0;
        }
    };

    let rust_params: RustMovementParams = params.into();
    
    let mut bridge = match handle.lock() {
        Ok(b) => b,
        Err(_) => {
            eprintln!("Failed to lock bridge handle");
            return 0;
        }
    };

    if let Some(robot_arc) = bridge.robot.clone() {
        let result = bridge.runtime.block_on(async move {
            let mut robot = robot_arc.lock().unwrap();
            robot.move_robot(rust_params).await
        });

        match result {
            Ok(_) => 1,
            Err(e) => {
                eprintln!("Failed to send movement command: {}", e);
                0
            }
        }
    } else {
        eprintln!("Robot not initialized");
        0
    }
}

#[no_mangle]
pub extern "C" fn rust_bridge_send_led_color(handle_id: i32, color: LedColor) -> c_bool {
    let handles = match HANDLES.lock() {
        Ok(h) => h,
        Err(_) => {
            eprintln!("Failed to lock handles mutex");
            return 0;
        }
    };

    let handle = match handles.get(handle_id as usize) {
        Some(Some(h)) => h.clone(),
        _ => {
            eprintln!("Invalid handle ID: {}", handle_id);
            return 0;
        }
    };

    let rust_color: RustLedColor = color.into();
    
    let mut bridge = match handle.lock() {
        Ok(b) => b,
        Err(_) => {
            eprintln!("Failed to lock bridge handle");
            return 0;
        }
    };

    if let Some(robot_arc) = bridge.robot.clone() {
        let result = bridge.runtime.block_on(async move {
            let mut robot = robot_arc.lock().unwrap();
            robot.control_led(rust_color).await
        });

        match result {
            Ok(_) => 1,
            Err(e) => {
                eprintln!("Failed to send LED color command: {}", e);
                0
            }
        }
    } else {
        eprintln!("Robot not initialized");
        0
    }
}

#[no_mangle]
pub extern "C" fn rust_bridge_stop(handle_id: i32) -> c_bool {
    let handles = match HANDLES.lock() {
        Ok(h) => h,
        Err(_) => {
            eprintln!("Failed to lock handles mutex");
            return 0;
        }
    };

    let handle = match handles.get(handle_id as usize) {
        Some(Some(h)) => h.clone(),
        _ => {
            eprintln!("Invalid handle ID: {}", handle_id);
            return 0;
        }
    };

    let mut bridge = match handle.lock() {
        Ok(b) => b,
        Err(_) => {
            eprintln!("Failed to lock bridge handle");
            return 0;
        }
    };

    if let Some(robot_arc) = bridge.robot.clone() {
        let result = bridge.runtime.block_on(async move {
            let mut robot = robot_arc.lock().unwrap();
            robot.stop().await
        });

        match result {
            Ok(_) => 1,
            Err(e) => {
                eprintln!("Failed to stop robot: {}", e);
                0
            }
        }
    } else {
        eprintln!("Robot not initialized");
        0
    }
}

#[no_mangle]
pub extern "C" fn rust_bridge_send_boot_sequence(handle_id: i32) -> c_bool {
    let handles = match HANDLES.lock() {
        Ok(h) => h,
        Err(_) => {
            eprintln!("Failed to lock handles mutex");
            return 0;
        }
    };

    let handle = match handles.get(handle_id as usize) {
        Some(Some(h)) => h.clone(),
        _ => {
            eprintln!("Invalid handle ID: {}", handle_id);
            return 0;
        }
    };

    let mut bridge = match handle.lock() {
        Ok(b) => b,
        Err(_) => {
            eprintln!("Failed to lock bridge handle");
            return 0;
        }
    };

    if let Some(robot_arc) = bridge.robot.clone() {
        let result = bridge.runtime.block_on(async move {
            let mut robot = robot_arc.lock().unwrap();
            robot.send_boot_sequence().await
        });

        match result {
            Ok(_) => 1,
            Err(e) => {
                eprintln!("Failed to send boot sequence: {}", e);
                0
            }
        }
    } else {
        eprintln!("Robot not initialized");
        0
    }
}

#[no_mangle]
pub extern "C" fn rust_bridge_get_last_error(_handle_id: i32) -> *const c_char {
    // For now, return a static error message
    // In a real implementation, you'd store per-handle error messages
    let error_msg = CString::new("Error details not available").unwrap();
    error_msg.into_raw()
}

#[no_mangle]
pub extern "C" fn rust_bridge_free_string(s: *mut c_char) {
    if !s.is_null() {
        unsafe {
            let _ = CString::from_raw(s);
        }
    }
}

#[no_mangle]
pub extern "C" fn rust_bridge_destroy(handle_id: i32) {
    let mut handles = match HANDLES.lock() {
        Ok(h) => h,
        Err(_) => {
            eprintln!("Failed to lock handles mutex");
            return;
        }
    };

    if let Some(slot) = handles.get_mut(handle_id as usize) {
        *slot = None;
    }
}

#[no_mangle]
pub extern "C" fn rust_bridge_read_sensor_data(handle_id: i32) -> c_bool {
    let handles = match HANDLES.lock() {
        Ok(h) => h,
        Err(_) => {
            eprintln!("Failed to lock handles mutex");
            return 0;
        }
    };

    let handle = match handles.get(handle_id as usize) {
        Some(Some(h)) => h.clone(),
        _ => {
            eprintln!("Invalid handle ID: {}", handle_id);
            return 0;
        }
    };

    let mut bridge = match handle.lock() {
        Ok(b) => b,
        Err(_) => {
            eprintln!("Failed to lock bridge handle");
            return 0;
        }
    };

    if let Some(robot_arc) = bridge.robot.clone() {
        let result = bridge.runtime.block_on(async move {
            let mut robot = robot_arc.lock().unwrap();
            robot.receive_sensor_data().await
        });

        match result {
            Ok(_) => 1,
            Err(e) => {
                eprintln!("Failed to receive sensor data: {}", e);
                0
            }
        }
    } else {
        eprintln!("Robot not initialized");
        0
    }
}

/// Get ESC (wheel motor) data
#[no_mangle]
pub extern "C" fn rust_bridge_get_esc_data(handle_id: i32, data: *mut EscData) -> c_bool {
    if data.is_null() {
        return 0;
    }

    let handles = match HANDLES.lock() {
        Ok(h) => h,
        Err(_) => return 0,
    };

    let handle = match handles.get(handle_id as usize) {
        Some(Some(h)) => h.clone(),
        _ => return 0,
    };

    let bridge = match handle.lock() {
        Ok(b) => b,
        Err(_) => return 0,
    };

    if let Some(ref sensor_state) = bridge.sensor_state {
        if let Ok(state) = sensor_state.read() {
            let esc = &state.esc;
            unsafe {
                (*data).speeds = esc.speeds_rad_s();
                (*data).angles = [
                    esc.angle_rad(0),
                    esc.angle_rad(1),
                    esc.angle_rad(2),
                    esc.angle_rad(3),
                ];
                (*data).has_data = if esc.has_data { 1 } else { 0 };
            }
            return 1;
        }
    }
    0
}

/// Get IMU data
#[no_mangle]
pub extern "C" fn rust_bridge_get_imu_data_new(handle_id: i32, data: *mut ImuData) -> c_bool {
    if data.is_null() {
        return 0;
    }

    let handles = match HANDLES.lock() {
        Ok(h) => h,
        Err(_) => return 0,
    };

    let handle = match handles.get(handle_id as usize) {
        Some(Some(h)) => h.clone(),
        _ => return 0,
    };

    let bridge = match handle.lock() {
        Ok(b) => b,
        Err(_) => return 0,
    };

    if let Some(ref sensor_state) = bridge.sensor_state {
        if let Ok(state) = sensor_state.read() {
            let imu = &state.imu;
            unsafe {
                (*data).accel = imu.accel;
                (*data).gyro = imu.gyro;
                (*data).has_data = if imu.has_data { 1 } else { 0 };
            }
            return 1;
        }
    }
    0
}

/// Get velocity data
#[no_mangle]
pub extern "C" fn rust_bridge_get_velocity_data(handle_id: i32, data: *mut VelocityData) -> c_bool {
    if data.is_null() {
        return 0;
    }

    let handles = match HANDLES.lock() {
        Ok(h) => h,
        Err(_) => return 0,
    };

    let handle = match handles.get(handle_id as usize) {
        Some(Some(h)) => h.clone(),
        _ => return 0,
    };

    let bridge = match handle.lock() {
        Ok(b) => b,
        Err(_) => return 0,
    };

    if let Some(ref sensor_state) = bridge.sensor_state {
        if let Ok(state) = sensor_state.read() {
            let velocity = &state.velocity;
            unsafe {
                (*data).body = velocity.body;
                (*data).has_data = if velocity.has_data { 1 } else { 0 };
            }
            return 1;
        }
    }
    0
}

/// Get position data
#[no_mangle]
pub extern "C" fn rust_bridge_get_position_data(handle_id: i32, data: *mut PositionData) -> c_bool {
    if data.is_null() {
        return 0;
    }

    let handles = match HANDLES.lock() {
        Ok(h) => h,
        Err(_) => return 0,
    };

    let handle = match handles.get(handle_id as usize) {
        Some(Some(h)) => h.clone(),
        _ => return 0,
    };

    let bridge = match handle.lock() {
        Ok(b) => b,
        Err(_) => return 0,
    };

    if let Some(ref sensor_state) = bridge.sensor_state {
        if let Ok(state) = sensor_state.read() {
            let position = &state.position;
            unsafe {
                (*data).x = position.x;
                (*data).y = position.y;
                (*data).z = position.z;
                (*data).has_data = if position.has_data { 1 } else { 0 };
            }
            return 1;
        }
    }
    0
}

/// Get attitude data
#[no_mangle]
pub extern "C" fn rust_bridge_get_attitude_data(handle_id: i32, data: *mut AttitudeData) -> c_bool {
    if data.is_null() {
        return 0;
    }

    let handles = match HANDLES.lock() {
        Ok(h) => h,
        Err(_) => return 0,
    };

    let handle = match handles.get(handle_id as usize) {
        Some(Some(h)) => h.clone(),
        _ => return 0,
    };

    let bridge = match handle.lock() {
        Ok(b) => b,
        Err(_) => return 0,
    };

    if let Some(ref sensor_state) = bridge.sensor_state {
        if let Ok(state) = sensor_state.read() {
            let attitude = &state.attitude;
            unsafe {
                (*data).yaw = attitude.yaw;
                (*data).pitch = attitude.pitch;
                (*data).roll = attitude.roll;
                (*data).has_data = if attitude.has_data { 1 } else { 0 };
            }
            return 1;
        }
    }
    0
}

/// Get battery level (legacy compatibility)
#[no_mangle]
pub extern "C" fn rust_bridge_get_battery_level(handle_id: i32) -> c_float {
    let handles = match HANDLES.lock() {
        Ok(h) => h,
        Err(_) => return 0.0,
    };

    let handle = match handles.get(handle_id as usize) {
        Some(Some(h)) => h.clone(),
        _ => return 0.0,
    };

    let bridge = match handle.lock() {
        Ok(b) => b,
        Err(_) => return 0.0,
    };

    if let Some(ref sensor_state) = bridge.sensor_state {
        if let Ok(state) = sensor_state.read() {
            return state.battery.adc as f32 / 1000.0; // Convert to voltage approximation
        }
    }
    0.0
}

/// Get IMU data (legacy compatibility)
#[no_mangle]
pub extern "C" fn rust_bridge_get_imu_data(
    handle_id: i32,
    acc_x: *mut c_float, acc_y: *mut c_float, acc_z: *mut c_float,
    gyro_x: *mut c_float, gyro_y: *mut c_float, gyro_z: *mut c_float
) -> c_bool {
    let handles = match HANDLES.lock() {
        Ok(h) => h,
        Err(_) => return 0,
    };

    let handle = match handles.get(handle_id as usize) {
        Some(Some(h)) => h.clone(),
        _ => return 0,
    };

    let bridge = match handle.lock() {
        Ok(b) => b,
        Err(_) => return 0,
    };

    if let Some(ref sensor_state) = bridge.sensor_state {
        if let Ok(state) = sensor_state.read() {
            let imu = &state.imu;
            if !acc_x.is_null() { unsafe { *acc_x = imu.accel[0]; } }
            if !acc_y.is_null() { unsafe { *acc_y = imu.accel[1]; } }
            if !acc_z.is_null() { unsafe { *acc_z = imu.accel[2]; } }
            if !gyro_x.is_null() { unsafe { *gyro_x = imu.gyro[0]; } }
            if !gyro_y.is_null() { unsafe { *gyro_y = imu.gyro[1]; } }
            if !gyro_z.is_null() { unsafe { *gyro_z = imu.gyro[2]; } }
            return if imu.has_data { 1 } else { 0 };
        }
    }
    0
}

/// Legacy sensor data getter to ensure SensorData struct is generated in header
#[no_mangle]
pub extern "C" fn rust_bridge_get_sensor_data_legacy(_handle_id: i32, _data: *mut SensorData) -> c_bool {
    0
}
