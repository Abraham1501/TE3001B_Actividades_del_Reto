/*
 * Copyright (c) 2019, Manchester Robotics Ltd.
 * All rights reserved.
 *
 * This software is provided AS-IS for the TE3001B Challenge
 * 
 * MOTOR NODE - Micro-ROS ESP32 Application
 * 
 * This ROS 2 node controls a DC motor via an L298N motor driver.
 * It subscribes to /cmd_pwm (Int16: -255 to +255) and outputs:
 * - PWM signal for motor speed control
 * - GPIO direction signals for motor direction control
 * 
 * Features:
 * - Subscribes to /cmd_pwm topic (motor speed command)
 * - Publishes motor feedback (/motor/rpm, /motor/encoder, /motor/state)
 * - Encoder-based speed measurement with interrupt handling
 * - Connection state machine for reliable Micro-ROS operation
 * - No potentiometer or button control (ROS commands only)
 */

// ======== Include necessary Micro-ROS and ESP32 libraries ========
#include <micro_ros_arduino.h>           // Micro-ROS library for ESP32
#include <rcl/rcl.h>                     // ROS 2 client library (Core)
#include <rcl/error_handling.h>          // ROS 2 error handling utilities
#include <rclc/rclc.h>                   // Micro-ROS client library
#include <rclc/executor.h>               // Executor to handle callbacks
#include <std_msgs/msg/int16.h>          // Int16 message type for PWM command
#include <std_msgs/msg/float32.h>        // Float32 message type for RPM feedback
#include <std_msgs/msg/int32.h>          // Int32 message type for encoder count
#include <rmw_microros/rmw_microros.h>   // Middleware functions for Micro-ROS
#include <stdio.h>                       // Standard I/O for debugging

// ======== GPIO Pin Definitions ========
// Motor Driver Pins (L298N)
#define IN1_GPIO 26      // Motor direction control 1
#define IN2_GPIO 25      // Motor direction control 2
#define PWM_GPIO 27      // PWM signal for speed control (ENA)

// Encoder Pins
#define PHASEA_GPIO 18   // Encoder phase A (rising edge interrupt)
#define PHASEB_GPIO 19   // Encoder phase B (direction detection)

// LED Status Indicator
#define LED_GPIO 2       // Status LED

// ======== PWM Configuration ========
#define PWM_FREQUENCY 1000   // PWM frequency in Hz
#define PWM_RESOLUTION 8     // 8-bit resolution (0-255)
#define PWM_CHANNEL 0        // Using PWM channel 0

// ======== Encoder Configuration ========
#define PULSES_PER_REV 495.0f    // Encoder pulses per revolution
#define SAMPLE_TIME_MS 100       // Control loop sample time (100ms)
#define RPM_MAX 110.0f           // Maximum motor RPM
#define CMD_TIMEOUT_MS 500       // Stop motor if no command received for this long (ms)

// ======== Micro-ROS Entity Declarations ========
rclc_support_t support;          // Micro-ROS execution context
rclc_executor_t executor;        // Manages execution of tasks
rcl_allocator_t allocator;       // Memory allocation

rcl_node_t node;                 // ROS 2 node

rcl_subscription_t cmd_pwm_subscriber;   // Subscribes to motor commands
rcl_publisher_t rpm_publisher;           // Publishes calculated RPM
rcl_publisher_t encoder_publisher;       // Publishes encoder count
rcl_publisher_t state_publisher;         // Publishes motor state
rcl_timer_t control_timer;               // Control loop timer

std_msgs__msg__Int16 cmd_pwm_msg;        // Motor command (-255 to +255)
std_msgs__msg__Float32 rpm_msg;          // Current RPM
std_msgs__msg__Int32 encoder_msg;        // Current encoder count
std_msgs__msg__Int16 state_msg;          // Motor state (0=stopped, 1=running)

// ======== Error Handling Macros ========
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Executes a statement (X) every N milliseconds
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

// ======== Motor Control Variables ========
volatile long encoderCountTotal = 0;          // Total encoder pulse count
volatile unsigned long lastCmdReceivedMs = 0; // Timestamp of last /cmd_pwm message

// Control loop state variables
long previousEncoderCount = 0;           // Encoder count from last sample
unsigned long previousMillis = 0;        // Timestamp of last control loop

// Current motor command (-255 to +255)
volatile int16_t currentPwmCommand = 0;
const int16_t PWM_CMD_MIN = -255;
const int16_t PWM_CMD_MAX = 255;

// ======== Micro-ROS Connection State Machine ========
enum states {
  WAITING_AGENT,        // Waiting for ROS 2 agent connection
  AGENT_AVAILABLE,      // Agent detected
  AGENT_CONNECTED,      // Successfully connected
  AGENT_DISCONNECTED    // Connection lost
} state;

// ======== Function Prototypes ========
bool create_entities();
void destroy_entities();
void apply_motor_command();

// ======== Interrupt Service Routine (ISR) - Encoder Phase A ========
// Called on rising edge of phase A only.
// Direction is determined by the state of phase B at that instant.
void IRAM_ATTR isrEncoderA() {
  if (digitalRead(PHASEB_GPIO)) {
    encoderCountTotal++;
  } else {
    encoderCountTotal--;
  }
}

// ======== Subscriber Callback: Receives Motor Command ========
// Processes incoming PWM command from /cmd_pwm topic
// Command range: -255 (reverse) to +255 (forward)
void cmd_pwm_callback(const void * msgin) {
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;
  
  // Constrain command to valid range and record timestamp for timeout check
  currentPwmCommand = (int16_t)constrain((int32_t)msg->data, PWM_CMD_MIN, PWM_CMD_MAX);
  lastCmdReceivedMs = millis();
}

// ======== Control Timer Callback ========
// Executes motor control loop at fixed sample time (100ms)
// - Calculates RPM from encoder pulses
// - Publishes feedback telemetry
void control_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  
  if (timer != NULL) {
    unsigned long now = millis();
    float deltaSec = (now - previousMillis) * 0.001f;
    
    // Safety: stop motor if no command received within timeout
    if (lastCmdReceivedMs > 0 && (now - lastCmdReceivedMs) > CMD_TIMEOUT_MS) {
      currentPwmCommand = 0;
    }

    // Read encoder values atomically
    long encoderCountNow = 0;
    noInterrupts();
    encoderCountNow = encoderCountTotal;
    interrupts();

    // Calculate RPM
    long pulsesInInterval = encoderCountNow - previousEncoderCount;
    float rpm = 0.0f;

    if (deltaSec > 0.0f) {
      rpm = (pulsesInInterval * 60.0f) / (PULSES_PER_REV * deltaSec);
    }

    // Update for next iteration
    previousEncoderCount = encoderCountNow;
    previousMillis = now;
    
    // Publish telemetry
    rpm_msg.data = rpm;
    encoder_msg.data = (int32_t)encoderCountNow;
    state_msg.data = (currentPwmCommand != 0) ? 1 : 0;  // 1=running, 0=stopped
    
    rcl_publish(&rpm_publisher, &rpm_msg, NULL);
    rcl_publish(&encoder_publisher, &encoder_msg, NULL);
    rcl_publish(&state_publisher, &state_msg, NULL);
    
    // LED toggle for status indication
    digitalWrite(LED_GPIO, !digitalRead(LED_GPIO));
  }
}

// ======== Apply Motor Command ========
// Translates PWM command to motor driver GPIO signals
// Positive: Forward (IN1=HIGH, IN2=LOW)
// Negative: Reverse (IN1=LOW, IN2=HIGH)
// Zero: Stop (IN1=HIGH, IN2=HIGH)
void apply_motor_command() {
  int16_t cmd = currentPwmCommand;
  
  // Determine direction and PWM duty cycle
  uint8_t pwmDuty = 0;

  if (cmd == 0) {
    // Brake: both pins HIGH on L298N activates short-circuit braking
    digitalWrite(IN1_GPIO, HIGH);
    digitalWrite(IN2_GPIO, HIGH);
    pwmDuty = 0;
  } else if (cmd > 0) {
    // Forward
    digitalWrite(IN1_GPIO, HIGH);
    digitalWrite(IN2_GPIO, LOW);
    pwmDuty = (uint8_t)constrain((int32_t)cmd, 0, 255);
  } else {
    // Reverse (cmd < 0)
    digitalWrite(IN1_GPIO, LOW);
    digitalWrite(IN2_GPIO, HIGH);
    pwmDuty = (uint8_t)constrain((int32_t)(-cmd), 0, 255);
  }
  
  // Apply PWM signal
  ledcWrite(PWM_CHANNEL, pwmDuty);
}

// ======== Setup Function ========
void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== Motor Node Starting ===\n");
  
  // Initialize Micro-ROS transport
  set_microros_transports();
  
  // Configure GPIO pins
  pinMode(IN1_GPIO, OUTPUT);
  pinMode(IN2_GPIO, OUTPUT);
  pinMode(LED_GPIO, OUTPUT);
  pinMode(PHASEA_GPIO, INPUT_PULLUP);
  pinMode(PHASEB_GPIO, INPUT_PULLUP);
  
  // Initialize motor to stopped state
  digitalWrite(IN1_GPIO, HIGH);
  digitalWrite(IN2_GPIO, HIGH);
  digitalWrite(LED_GPIO, LOW);
  
  // Setup PWM for motor speed control
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_GPIO, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);  // Start with zero PWM
  
  // Attach interrupt for encoder phase A - RISING only matches the ISR logic
  attachInterrupt(digitalPinToInterrupt(PHASEA_GPIO), isrEncoderA, RISING);
  
  // Initialize timing
  previousMillis = millis();
  
  // Set initial state to WAITING_AGENT
  state = WAITING_AGENT;
  
  Serial.println("Setup complete. Waiting for ROS 2 agent...\n");
}

// ======== Main Loop Function ========
void loop() {
  switch (state) {

    case WAITING_AGENT:
      // Periodically ping agent to check if it's available
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      // Agent detected, attempt to create ROS entities
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      }
      if (state == AGENT_CONNECTED) {
        Serial.println("Connected to ROS 2 agent!");
      }
      break;

    case AGENT_CONNECTED:
      // Periodically check if agent is still connected
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      
      if (state == AGENT_CONNECTED) {
        // Apply current motor command
        apply_motor_command();
        
        // Execute ROS executor to handle callbacks
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;

    case AGENT_DISCONNECTED:
      // Connection lost, cleanup and wait for reconnection
      Serial.println("Agent disconnected. Cleaning up...");
      destroy_entities();
      currentPwmCommand = 0;
      apply_motor_command();  // Stop motor
      state = WAITING_AGENT;
      break;
      
    default:
      break;
  }
}

// ======== ROS 2 Entity Creation ========
bool create_entities() {
  Serial.println("Creating ROS 2 entities...");
  
  // Initialize Micro-ROS support
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // Create node with name "motor_node"
  RCCHECK(rclc_node_init_default(&node, "motor_node", "", &support));

  // Initialize subscriber for motor command
  RCCHECK(rclc_subscription_init_default(
      &cmd_pwm_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "cmd_pwm"));

  // Initialize publishers for feedback
  RCCHECK(rclc_publisher_init_default(
      &rpm_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "motor/rpm"));

  RCCHECK(rclc_publisher_init_default(
      &encoder_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "motor/encoder"));

  RCCHECK(rclc_publisher_init_default(
      &state_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "motor/state"));

  // Initialize control timer (100ms period)
  const unsigned int timer_timeout = SAMPLE_TIME_MS;
  RCCHECK(rclc_timer_init_default(
      &control_timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      control_timer_callback));

  // Initialize executor with 2 handles (1 subscription + 1 timer)
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  
  // Add subscription and timer to executor
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_pwm_subscriber, &cmd_pwm_msg, &cmd_pwm_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

  Serial.println("ROS 2 entities created successfully!");
  return true;
}

// ======== ROS 2 Entity Cleanup ========
void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&cmd_pwm_subscriber, &node);
  rcl_publisher_fini(&rpm_publisher, &node);
  rcl_publisher_fini(&encoder_publisher, &node);
  rcl_publisher_fini(&state_publisher, &node);
  rcl_timer_fini(&control_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}
