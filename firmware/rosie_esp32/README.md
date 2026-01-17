# Rosie ESP32 - Micro-ROS Robot Control Firmware

Complete Micro-ROS firmware for ESP32S3 robot with odometry publishing for ROS2 Nav2 autonomous navigation.

## Features

- **Micro-ROS Integration** - Full ROS2 support via micro-ROS
- **Complete Odometry** - Publishes `nav_msgs/Odometry` for Nav2 navigation
- **4-Wheel Drive** - Motor control with PWM
- **IMU Sensor** - ICM42670P gyro + accelerometer
- **Encoder Support** - Encoder with ISR
- **Servo & Buzzer** - Additional hardware support

## Hardware

- **MCU**: ESP32-S3 (Adafruit Feather ESP32-S3)
- **Motors**: 4x DC Motors with H-bridge driver
- **Encoders**: Quadrature encoders
- **IMU**: ICM42670P (I2C)
- **Servo**: 2x Servo motors

## Quick Start

### Build & Upload
```bash
platformio run -e esp32s3_4MB --target upload
```

### Connect to ROS2
```bash
# Start micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# Verify topics
ros2 topic list
# Should show: /wheel/odometry, /imu/data, /encoder, /cmd_vel
```

### Test Drive
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0}}"

# Monitor odometry
ros2 topic echo /wheel/odometry
```

## Configuration

Edit robot parameters in `src/main.cpp`:

```cpp
const float WHEEL_DIAMETER = 0.065f;        // meters (65mm wheels)
const float TICKS_PER_REVOLUTION = 360.0f;  // encoder ticks per rotation
const float ROBOT_WIDTH = 0.17f;            // meters (170mm wheelbase)
const float ROBOT_LENGTH = 0.285f;          // meters (285mm base length)
```

**Rosie Robot Dimensions:**
- Base footprint: 0.285m × 0.17m
- Safety margin for Nav2: 0.25m padding

## Published Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/wheel/odometry` | nav_msgs/Odometry | Raw wheel odometry for the Rosie EKF |
| `/imu/data` | sensor_msgs/Imu | IMU sensor data for the Rosie EKF |
| `/encoder` | std_msgs/Int16 | Raw encoder ticks |

## Subscribed Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/cmd_vel` | geometry_msgs/Twist | Motor velocity commands |
| `/buzzer` | std_msgs/Int16 | Buzzer control (0-255) |

## Pin Configuration

| Component | Pins |
|-----------|------|
| Motor 1 PWM | 4, 5 |
| Motor 2 PWM | 15, 16 |
| Motor 3 PWM | 10, 9 |
| Motor 4 PWM | 14, 13 |
| Encoder A/B | 2, 1 |
| IMU (I2C) | SDA=40, SCL=39 |
| Servo 1/2 | 8, 21 |
| Buzzer | 46 |
| Status LEDs | 45, 38 |



