/*
 * Copyright (c) 2019, Manchester Robotics Ltd.
 * All rights reserved.
 *
 * This software is provided AS-IS for the TE3001B Challenge
 *
 * MOTOR POSITION NODE – Micro-ROS ESP32 Application
 *
 * Closed-loop PD position controller for a DC motor with quadrature encoder.
 * Designed to drive a "clock hand" simulation: the ROS 2 side translates
 * clock-face positions (hours / half-hours) into absolute encoder targets
 * and publishes them on /cmd_pos.
 *
 * Topics subscribed
 *   /cmd_pos          (std_msgs/Int32)   – absolute encoder-pulse target
 *   /clock_reset      (std_msgs/Bool)    – reset encoder and target to home
 *
 * Topics published
 *   /motor/encoder    (std_msgs/Int32)   – live absolute encoder count
 *   /motor/pos_error  (std_msgs/Float32) – signed position error [pulses]
 *   /motor/rpm        (std_msgs/Float32) – filtered motor RPM
 *   /motor/state      (std_msgs/Int16)   – 0 = at rest / 1 = moving
 *
 * Clock geometry (hard-coded, tune if gearbox differs):
 *   PULSES_PER_REV  = 495   → one full clock revolution
 *   PULSES_PER_HOUR = 495/12 = 41.25  pulses
 *   PULSES_PER_HALF = 495/24 = 20.625 pulses
 *
 * Example:  "10 complete turns then 5 o'clock"
 *   target = 10 × 495  +  5 × 41.25  = 5156 pulses
 *   → publish Int32 data: 5156  to /cmd_pos
 */

// ======== Micro-ROS / ROS 2 libraries ========
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <rmw_microros/rmw_microros.h>
#include <stdio.h>
#include <math.h>

#define IN1_GPIO     26   // L298N direction pin 1
#define IN2_GPIO     25   // L298N direction pin 2
#define PWM_GPIO     27   // L298N ENA (PWM)
#define PHASEA_GPIO  18   // Encoder channel A  (interrupt)
#define PHASEB_GPIO  19   // Encoder channel B  (direction)
#define LED_GPIO      2   // Status LED

#define PWM_FREQUENCY  1000   // Hz
#define PWM_RESOLUTION    8   // bits  (0 – 255)
#define PWM_CHANNEL       0

#define PULSES_PER_REV    495.0f   // Encoder pulses per shaft revolution
#define SAMPLE_TIME_MS     50      // Control-loop period  (50 ms = 20 Hz)

// Each half-hour  = PULSES_PER_REV / 24
#define PULSES_PER_HOUR   (PULSES_PER_REV / 12.0f)   // ≈ 41.25 pulses
#define PULSES_PER_HALF   (PULSES_PER_REV / 24.0f)   // ≈ 20.625 pulses

// Tune these for your specific motor/load combination.
#define PD_KP  1.2f
#define PD_KD  8.0f

// ======== Position Controller Limits ========
#define POSITION_TOLERANCE_PULSES  2     // Dead-band: stop if |error| ≤ this
#define MIN_EFFECTIVE_PWM          60     // Minimum PWM to overcome static friction
#define PWM_OUTPUT_MAX             255    // Maximum PWM output

// ======== RPM Filter ========
#define RPM_FILTER_ALPHA  0.3f   // EMA alpha (0 < α ≤ 1; larger = more responsive)

// ======== Micro-ROS Objects ========
rclc_support_t   support;
rclc_executor_t  executor;
rcl_allocator_t  allocator;
rcl_node_t       node;

rcl_subscription_t  cmd_pos_subscriber;
rcl_subscription_t  reset_subscriber;
rcl_publisher_t     encoder_publisher;
rcl_publisher_t     pos_error_publisher;
rcl_publisher_t     rpm_publisher;
rcl_publisher_t     state_publisher;
rcl_timer_t         control_timer;

std_msgs__msg__Int32   cmd_pos_msg;
std_msgs__msg__Bool    reset_msg;
std_msgs__msg__Int32   encoder_msg;
std_msgs__msg__Float32 pos_error_msg;
std_msgs__msg__Float32 rpm_msg;
std_msgs__msg__Int16   state_msg;

// ======== Error-handling Macros ========
#define RCCHECK(fn) { rcl_ret_t _rc = (fn); if (_rc != RCL_RET_OK) { print_rcl_error(#fn, _rc); return false; } }
#define RCSOFTCHECK(fn) { rcl_ret_t _rc = (fn); (void)_rc; }

#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t _init = -1; \
  if (_init == -1) { _init = uxr_millis(); } \
  if (uxr_millis() - _init > (MS)) { X; _init = uxr_millis(); } \
} while (0)

// ======== State Variables ========
volatile long  encoderCountTotal = 0;   // ISR-updated absolute pulse count

long     previousEncoderCount = 0;      // For RPM calculation
unsigned long previousMillis   = 0;
float    filteredRpm           = 0.0f;

// Position controller
volatile int32_t targetEncoderCount  = 0;   // Commanded absolute position [pulses]
float            prevPosError        = 0.0f; // Previous error for derivative term
bool             positionCommandNew  = false; // Flag: new /cmd_pos received

// ======== Connection State Machine ========
enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;

bool create_entities();
void destroy_entities();
void apply_motor_command(int16_t cmd);
void print_rcl_error(const char * fn_name, rcl_ret_t rc);
void reset_position_state();

// ======== ISR: Encoder Phase A (rising edge) ========
void IRAM_ATTR isrEncoderA() {
  encoderCountTotal += digitalRead(PHASEB_GPIO) ? 1 : -1;
}

// Receives the absolute encoder-pulse target published by the ROS 2 clock node.
// Example: to reach "5 o'clock after 10 full revolutions" publish 5156.
void cmd_pos_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  targetEncoderCount = msg->data;
  positionCommandNew = true;

  Serial.print("[RX] /cmd_pos = ");
  Serial.println(targetEncoderCount);
}

void reset_position_callback(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  if (!msg->data) {
    return;
  }

  reset_position_state();
  Serial.println("[RX] /clock_reset = true -> encoder and target reset to 0");
}

void reset_position_state() {
  noInterrupts();
  encoderCountTotal = 0;
  interrupts();

  targetEncoderCount = 0;
  previousEncoderCount = 0;
  filteredRpm = 0.0f;
  prevPosError = 0.0f;
  positionCommandNew = false;

  apply_motor_command(0);
}

void print_rcl_error(const char * fn_name, rcl_ret_t rc) {
  Serial.print("[RCL ERROR] ");
  Serial.print(fn_name);
  Serial.print(" failed with code ");
  Serial.println((int)rc);

  const rcl_error_string_t error_string = rcl_get_error_string();
  if (error_string.str != NULL) {
    Serial.print("[RCL ERROR] message: ");
    Serial.println(error_string.str);
  }
  rcl_reset_error();
}

// ======== Control Timer Callback (runs every SAMPLE_TIME_MS) ========
void control_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) return;

  unsigned long now = millis();
  float dt = (now - previousMillis) * 0.001f;   // seconds

  // ---- Read encoder atomically ----
  long encoderNow = 0;
  noInterrupts();
  encoderNow = encoderCountTotal;
  interrupts();

  // ---- RPM (for monitoring) ----
  long pulsesInInterval = encoderNow - previousEncoderCount;
  float rawRpm = (dt > 0.0f) ? (pulsesInInterval * 60.0f) / (PULSES_PER_REV * dt) : 0.0f;

  if (rawRpm == 0.0f) {
    filteredRpm = 0.0f;
  } else {
    filteredRpm = RPM_FILTER_ALPHA * rawRpm + (1.0f - RPM_FILTER_ALPHA) * filteredRpm;
  }

  previousEncoderCount = encoderNow;
  previousMillis       = now;

  // ---- PD Position Controller ----
  int32_t target = targetEncoderCount;
  float error    = (float)(target - encoderNow);

  int16_t pwmCmd = 0;

  if (fabsf(error) <= (float)POSITION_TOLERANCE_PULSES) {
    // Inside dead-band → hold still
    pwmCmd = 0;
    prevPosError = 0.0f;   // reset derivative memory when at rest
  } else {
    // PD law:  u = Kp·e + Kd·Δe
    float dError = error - prevPosError;
    float u      = PD_KP * error + PD_KD * dError;

    // Clamp to valid PWM range
    u = constrain(u, (float)-PWM_OUTPUT_MAX, (float)PWM_OUTPUT_MAX);

    // Enforce minimum effective PWM to overcome static friction
    if (u > 0.0f && u < MIN_EFFECTIVE_PWM) u = (float)MIN_EFFECTIVE_PWM;
    if (u < 0.0f && u > -MIN_EFFECTIVE_PWM) u = (float)-MIN_EFFECTIVE_PWM;

    pwmCmd = (int16_t)u;
    prevPosError = error;
  }

  apply_motor_command(pwmCmd);

  // ---- Publish Feedback ----
  encoder_msg.data   = (int32_t)encoderNow;
  pos_error_msg.data = error;
  rpm_msg.data       = filteredRpm;
  state_msg.data     = (pwmCmd != 0) ? 1 : 0;

  rcl_publish(&encoder_publisher,   &encoder_msg,   NULL);
  rcl_publish(&pos_error_publisher, &pos_error_msg, NULL);
  rcl_publish(&rpm_publisher,       &rpm_msg,       NULL);
  rcl_publish(&state_publisher,     &state_msg,     NULL);

  // LED heartbeat
  digitalWrite(LED_GPIO, !digitalRead(LED_GPIO));
}

// Positive cmd  → forward  (IN1=HIGH, IN2=LOW)
// Negative cmd  → reverse  (IN1=LOW,  IN2=HIGH)
// Zero cmd      → brake    (IN1=HIGH, IN2=HIGH)
void apply_motor_command(int16_t cmd) {
  uint8_t duty = 0;

  if (cmd == 0) {
    digitalWrite(IN1_GPIO, HIGH);
    digitalWrite(IN2_GPIO, HIGH);
    duty = 0;
  } else if (cmd > 0) {
    digitalWrite(IN1_GPIO, HIGH);
    digitalWrite(IN2_GPIO, LOW);
    duty = (uint8_t)constrain((int32_t)cmd, 0, 255);
  } else {
    digitalWrite(IN1_GPIO, LOW);
    digitalWrite(IN2_GPIO, HIGH);
    duty = (uint8_t)constrain((int32_t)(-cmd), 0, 255);
  }
  ledcWrite(PWM_CHANNEL, duty);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== Motor Position Node Starting ===");
  Serial.println("Firmware: position control + Micro-ROS diagnostics");
  Serial.printf("  PULSES_PER_REV  = %.1f\n",  PULSES_PER_REV);
  Serial.printf("  PULSES_PER_HOUR = %.3f\n",  PULSES_PER_HOUR);
  Serial.printf("  PULSES_PER_HALF = %.3f\n",  PULSES_PER_HALF);
  Serial.printf("  Kp=%.2f  Kd=%.2f  tol=%d pulses\n",
                PD_KP, PD_KD, POSITION_TOLERANCE_PULSES);
  Serial.println("Expected ROS entities:");
  Serial.println("  Subscriber: /cmd_pos (std_msgs/Int32)");
  Serial.println("  Subscriber: /clock_reset (std_msgs/Bool)");
  Serial.println("  Publishers: /motor/encoder, /motor/pos_error, /motor/rpm, /motor/state");

  set_microros_transports();
  Serial.println("Micro-ROS transport configured.");

  pinMode(IN1_GPIO,    OUTPUT);
  pinMode(IN2_GPIO,    OUTPUT);
  pinMode(LED_GPIO,    OUTPUT);
  pinMode(PHASEA_GPIO, INPUT_PULLUP);
  pinMode(PHASEB_GPIO, INPUT_PULLUP);

  // Brake at startup
  digitalWrite(IN1_GPIO, HIGH);
  digitalWrite(IN2_GPIO, HIGH);
  digitalWrite(LED_GPIO, LOW);

  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_GPIO, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);

  attachInterrupt(digitalPinToInterrupt(PHASEA_GPIO), isrEncoderA, RISING);

  previousMillis = millis();
  state = WAITING_AGENT;

  Serial.println("Setup complete – waiting for ROS 2 agent …\n");
}

void loop() {
  switch (state) {

    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(2000,
        Serial.println("[STATE] WAITING_AGENT - pinging agent...");
      );
      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                ? AGENT_AVAILABLE : WAITING_AGENT;
      );
      break;

    case AGENT_AVAILABLE:
      Serial.println("[STATE] AGENT_AVAILABLE - creating ROS entities...");
      state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        Serial.println("[STATE] Entity creation failed. Returning to WAITING_AGENT.");
        destroy_entities();
      }
      if (state == AGENT_CONNECTED) {
        Serial.println("[STATE] Connected to ROS 2 agent!");
        Serial.println("[STATE] Position subscribers /cmd_pos and /clock_reset should now be visible.");
      }
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );
      if (state == AGENT_CONNECTED) {
        EXECUTE_EVERY_N_MS(3000,
          Serial.print("[STATE] CONNECTED | encoder=");
          Serial.print(encoderCountTotal);
          Serial.print(" target=");
          Serial.print(targetEncoderCount);
          Serial.print(" error=");
          Serial.println((long)(targetEncoderCount - encoderCountTotal));
        );
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;

    case AGENT_DISCONNECTED:
      Serial.println("Agent disconnected – cleaning up …");
      destroy_entities();
      apply_motor_command(0);   // brake
      state = WAITING_AGENT;
      break;

    default: break;
  }
}

// ======== ROS 2 Entity Creation ========
bool create_entities() {
  Serial.println("Creating ROS 2 entities …");

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "motor_position_node", "", &support));

  // Subscriber: absolute position target [pulses]
  RCCHECK(rclc_subscription_init_default(
      &cmd_pos_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "cmd_pos"));

    RCCHECK(rclc_subscription_init_default(
      &reset_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "clock_reset"));

  // Publishers
  RCCHECK(rclc_publisher_init_default(
      &encoder_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "motor/encoder"));

  RCCHECK(rclc_publisher_init_default(
      &pos_error_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "motor/pos_error"));

  RCCHECK(rclc_publisher_init_default(
      &rpm_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "motor/rpm"));

  RCCHECK(rclc_publisher_init_default(
      &state_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "motor/state"));

  // Control timer: fires every SAMPLE_TIME_MS milliseconds
  RCCHECK(rclc_timer_init_default(
      &control_timer, &support,
      RCL_MS_TO_NS(SAMPLE_TIME_MS),
      control_timer_callback));

  // Executor: 2 subscriptions + 1 timer = 3 handles
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &cmd_pos_subscriber, &cmd_pos_msg, &cmd_pos_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &reset_subscriber, &reset_msg, &reset_position_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

  Serial.println("ROS 2 entities created successfully!");
  Serial.println("You can verify from the PC with: ros2 topic info /cmd_pos");
  Serial.println("Reset from terminal with: ros2 topic pub --once /clock_reset std_msgs/msg/Bool '{data: true}'");
  return true;
}

// ======== ROS 2 Entity Cleanup ========
void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&cmd_pos_subscriber, &node);
  rcl_subscription_fini(&reset_subscriber,   &node);
  rcl_publisher_fini(&encoder_publisher,     &node);
  rcl_publisher_fini(&pos_error_publisher,   &node);
  rcl_publisher_fini(&rpm_publisher,         &node);
  rcl_publisher_fini(&state_publisher,       &node);
  rcl_timer_fini(&control_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}
