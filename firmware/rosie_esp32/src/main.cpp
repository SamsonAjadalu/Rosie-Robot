/*not
 * Micro-ROS ESP32 Complete Code for Robot Control with Motor, IMU, Encoder, Servos, and Buzzer
 */

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

// ROS Message Types
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.h>

// Hardware Libraries
#include <ESP32Servo.h>
#include <Wire.h>
#include <ICM42670P.h>

// Motor Control Pins and PWM Settings
#define MOTOR1_PWM_A 4
#define MOTOR1_PWM_B 5
#define MOTOR2_PWM_A 15
#define MOTOR2_PWM_B 16
#define MOTOR3_PWM_A 10
#define MOTOR3_PWM_B 9
#define MOTOR4_PWM_A 14
#define MOTOR4_PWM_B 13
#define PWM_FREQ 10000
#define PWM_RESOLUTION 8
#define MOTOR_SPEED 128

// Encoder Pins
#define encoderPinA 2
#define encoderPinB 1
volatile long encoderCount = 0;

// IMU Pins
#define SDA_PIN 40
#define SCL_PIN 39
#define IMU_INT_PIN 41
ICM42670 IMU(Wire, 0);

// Servo Pins
Servo servo1;
Servo servo2;

// Buzzer Pin
#define BUZZER_PIN 46

// LED Pins for Status Indication
#define LED_PIN 45
#define LED_PIN_TEST 38
bool led_test_state = false;

#define EXECUTE_EVERY_N_MS(MS, X) do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)

// ROS Entities
rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t imu_pub, encoder_pub, odom_pub;
rcl_subscription_t cmd_vel_sub, buzzer_sub;

// Messages
std_msgs__msg__Int16 encoder_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__Int16 buzzer_msg;
nav_msgs__msg__Odometry odom_msg;

// Odometry Calculation Variables
// Robot Parameters - ADJUST THESE FOR YOUR ROBOT
const float WHEEL_DIAMETER = 0.065f;  // meters (65mm wheel)
const float TICKS_PER_REVOLUTION = 360.0f;  // encoder ticks per full wheel rotation
const float WHEEL_CIRCUMFERENCE = 3.14159f * WHEEL_DIAMETER;
const float DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_REVOLUTION;
const float ROBOT_WIDTH = 0.17f;  // meters (170mm wheelbase - actual robot dimension)
const float ROBOT_LENGTH = 0.285f;  // meters (285mm base length)
const uint32_t ODOM_PUBLISH_INTERVAL_MS = 100;  // Publish odometry every 100ms

// Odometry State
long lastEncoderCount = 0;
float odometry_x = 0.0f;
float odometry_y = 0.0f;
float odometry_theta = 0.0f;  // radians
float linear_velocity = 0.0f;
float angular_velocity = 0.0f;
int64_t lastOdomTime = 0;

// State Management
enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;

// Motor Control Function
void moveMotors(int motor1, int motor2, int motor3, int motor4) {
  ledcWrite(MOTOR1_PWM_A, motor1 > 0 ? motor1 : 0);
  ledcWrite(MOTOR1_PWM_B, motor1 < 0 ? -motor1 : 0);
  ledcWrite(MOTOR2_PWM_A, motor2 > 0 ? motor2 : 0);
  ledcWrite(MOTOR2_PWM_B, motor2 < 0 ? -motor2 : 0);
  ledcWrite(MOTOR3_PWM_A, motor3 > 0 ? motor3 : 0);
  ledcWrite(MOTOR3_PWM_B, motor3 < 0 ? -motor3 : 0);
  ledcWrite(MOTOR4_PWM_A, motor4 > 0 ? motor4 : 0);
  ledcWrite(MOTOR4_PWM_B, motor4 < 0 ? -motor4 : 0);
}

// Command Velocity Callback
void cmdVelCallback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  int motor_speed = (int)(255 * msg->linear.x);
  moveMotors(motor_speed, motor_speed, motor_speed, motor_speed);
}

// Buzzer Control Callback
void buzzerCallback(const void *msgin) {
  const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *)msgin;
  ledcWrite(BUZZER_PIN, msg->data);
}

// Calculate Odometry from Encoder Data
void calculateOdometry(int64_t current_time) {
  long tick_delta = encoderCount - lastEncoderCount;
  lastEncoderCount = encoderCount;
  
  if (lastOdomTime == 0) {
    lastOdomTime = current_time;
    return;
  }
  
  // Calculate time delta in seconds
  float dt = (current_time - lastOdomTime) / 1000000.0f;  // Convert microseconds to seconds
  if (dt <= 0) dt = 0.001f;  // Avoid division by zero
  lastOdomTime = current_time;
  
  // Calculate distance traveled
  float distance = tick_delta * DISTANCE_PER_TICK;
  
  // Update position (simplified: assuming straight line motion)
  // For full 4-wheel odometry, sum all wheels and divide by 4
  odometry_x += distance * cosf(odometry_theta);
  odometry_y += distance * sinf(odometry_theta);
  
  // Calculate velocities
  linear_velocity = distance / dt;
  // Angular velocity would come from IMU gyro in a complete implementation
}

// Quaternion from Euler angle (roll=0, pitch=0, yaw=theta)
void eulerToQuaternion(float theta, float &qx, float &qy, float &qz, float &qw) {
  float half_theta = theta / 2.0f;
  qx = 0.0f;
  qy = 0.0f;
  qz = sinf(half_theta);
  qw = cosf(half_theta);
}

// Publish Odometry Message for Nav2
void publishOdometry(int64_t current_time) {
  // Set frame IDs
  odom_msg.header.frame_id.data = (char *)"odom";
  odom_msg.header.frame_id.size = 4;
  odom_msg.child_frame_id.data = (char *)"base_link";
  odom_msg.child_frame_id.size = 9;
  
  // Set timestamp
  odom_msg.header.stamp.sec = current_time / 1000000000LL;
  odom_msg.header.stamp.nanosec = current_time % 1000000000LL;
  
  // Set position
  odom_msg.pose.pose.position.x = odometry_x;
  odom_msg.pose.pose.position.y = odometry_y;
  odom_msg.pose.pose.position.z = 0.0f;
  
  // Set orientation (quaternion from theta)
  float qx, qy, qz, qw;
  eulerToQuaternion(odometry_theta, qx, qy, qz, qw);
  odom_msg.pose.pose.orientation.x = qx;
  odom_msg.pose.pose.orientation.y = qy;
  odom_msg.pose.pose.orientation.z = qz;
  odom_msg.pose.pose.orientation.w = qw;
  
  // Set velocity
  odom_msg.twist.twist.linear.x = linear_velocity;
  odom_msg.twist.twist.linear.y = 0.0f;
  odom_msg.twist.twist.linear.z = 0.0f;
  odom_msg.twist.twist.angular.x = 0.0f;
  odom_msg.twist.twist.angular.y = 0.0f;
  odom_msg.twist.twist.angular.z = angular_velocity;
  
  // Publish
  rcl_publish(&odom_pub, &odom_msg, NULL);
}

// Timer Callback for IMU, Encoder, and Odometry Publishing
void timerCallback(rcl_timer_t *timer, int64_t last_call_time) {
  if (timer != NULL) {
    int64_t current_time = uxr_millis() * 1000000LL;  // Convert to nanoseconds
    
    // Calculate odometry from encoder
    calculateOdometry(current_time);
    
    // Publish Odometry Data (for Nav2)
    publishOdometry(current_time);
    
    // Publish Encoder Data
    encoder_msg.data = encoderCount;
    rcl_publish(&encoder_pub, &encoder_msg, NULL);

    // Publish IMU Data
    inv_imu_sensor_event_t imu_event;
    if (IMU.getDataFromRegisters(imu_event) == 0) {
      imu_msg.linear_acceleration.x = imu_event.accel[0];
      imu_msg.linear_acceleration.y = imu_event.accel[1];
      imu_msg.linear_acceleration.z = imu_event.accel[2];
      imu_msg.angular_velocity.x = imu_event.gyro[0];
      imu_msg.angular_velocity.y = imu_event.gyro[1];
      imu_msg.angular_velocity.z = imu_event.gyro[2];
      rcl_publish(&imu_pub, &imu_msg, NULL);
    }
  }
}

// Initialize ROS Node, Publishers, and Subscriptions
bool create_entities() {
  allocator = rcl_get_default_allocator();
  rcl_init_options_init(&init_options, allocator);
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rclc_node_init_default(&node, "esp32_node", "", &support);

  // Initialize Publishers
  rclc_publisher_init_default(&encoder_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/encoder");
  // Publish raw sensor streams on the topics consumed by rosie_localization.
  // The EKF publishes the fused /odom topic for Nav2.
  rclc_publisher_init_default(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data");
  rclc_publisher_init_default(&odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/wheel/odometry");

  // Initialize Subscriptions
  rclc_subscription_init_default(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
  rclc_subscription_init_default(&buzzer_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/buzzer");

  // Timer Initialization
  const unsigned int timer_timeout = 100; // ms
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timerCallback);

  // Executor Initialization
  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, cmdVelCallback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &buzzer_sub, &buzzer_msg, buzzerCallback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  return true;
}

// Clean Up ROS Entities
void destroy_entities() {
  rcl_publisher_fini(&imu_pub, &node);
  rcl_publisher_fini(&encoder_pub, &node);
  rcl_publisher_fini(&odom_pub, &node);
  rcl_subscription_fini(&cmd_vel_sub, &node);
  rcl_subscription_fini(&buzzer_sub, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// Setup Function
void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  // Motor PWM Configuration
  ledcAttachPin(MOTOR1_PWM_A, 0);
  ledcAttachPin(MOTOR1_PWM_B, 1);
  ledcAttachPin(MOTOR2_PWM_A, 2);
  ledcAttachPin(MOTOR2_PWM_B, 3);
  ledcAttachPin(MOTOR3_PWM_A, 4);
  ledcAttachPin(MOTOR3_PWM_B, 5);
  ledcAttachPin(MOTOR4_PWM_A, 6);
  ledcAttachPin(MOTOR4_PWM_B, 7);
  // Set all motor PWM values to zero to stop them

    ledcWrite(0, 0);
    ledcWrite(1, 0);
    ledcWrite(2, 0);
    ledcWrite(3, 0);
    ledcWrite(4, 0);
    ledcWrite(5, 0);
    ledcWrite(6, 0);
    ledcWrite(7, 0);
  

  // Encoder Configuration
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, RISING);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN_TEST, OUTPUT);

  // IMU Initialization
  Wire.begin(SDA_PIN, SCL_PIN);
  IMU.begin();
  IMU.startAccel(100, 16);
  IMU.startGyro(100, 2000);

  // Servo Initialization
  servo1.attach(8); // Pin for Servo 1
  servo2.attach(21); // Pin for Servo 2

  // Buzzer Initialization
  ledcAttachPin(BUZZER_PIN, PWM_FREQ);

  // State Initialization
  state = WAITING_AGENT;
}

// Main Loop
void loop() {
  ledcWrite(0, 128);
  ledcWrite(1, 0);
    switch (state) {
        case WAITING_AGENT:
            // Ping the agent every 500 ms to check availability
            EXECUTE_EVERY_N_MS(500, 
                state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) 
                        ? AGENT_AVAILABLE : WAITING_AGENT;
            );
            break;

        case AGENT_AVAILABLE:
            // Try to create entities; if successful, connect
            state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) {
                destroy_entities(); // Clean up if entity creation fails
            }
            break;

        case AGENT_CONNECTED:
            // Check connection status every 200 ms
            EXECUTE_EVERY_N_MS(200, 
                if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
                    state = AGENT_DISCONNECTED;
                }
            );
            if (state == AGENT_CONNECTED) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            break;

        case AGENT_DISCONNECTED:
            // Restart ESP32 if disconnected
            ESP.restart();
            break;
    }

    // Set LED to indicate connection status
    digitalWrite(LED_PIN, state == AGENT_CONNECTED ? HIGH : LOW);
}
