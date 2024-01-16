#include <micro_ros_platformio.h>
#include <Arduino.h>
#include <stdio.h>

#include <config_drive_input.h>
#include <kinematics.h>
#include <pid.h>
#include <odometry.h>
// #include <ESP32Encoder.h>
#include <encoder.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int8.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      return false;              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }
#define EXECUTE_EVERY_N_MS(MS, X)      \
  do                                   \
  {                                    \
    static volatile int64_t init = -1; \
    if (init == -1)                    \
    {                                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS)      \
    {                                  \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)

//------------------------------ < ESP32 Define > -----------------------------------//
unsigned long long time_offset = 0;
unsigned long prev_velocity_time = 0;
unsigned long prev_odom_update = 0;

// ESP32Encoder motor1_encoder(COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
// ESP32Encoder motor2_encoder(COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
// ESP32Encoder motor3_encoder(COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(
    Kinematics::MECANUM,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE);

Odometry odometry;
bool isPowered = false;
int pre_power = 1;

//------------------------------ < Fuction Prototype > ------------------------------//
void moveBase();
void syncTime();
void publishData();
struct timespec getTime();
int lim_switch(int lim_pin);

//------------------------------ < Ros Fuction Prototype > --------------------------//

void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void sub_velocity_callback(const void *msgin);
bool create_entities();
void destroy_entities();
void renew();

//------------------------------ < Ros Define > -------------------------------------//
// basic
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rcl_allocator_t allocator;
rclc_executor_t executor;

// ? define msg
std_msgs__msg__Int8 start_msg;
std_msgs__msg__Int8 team_msg;
std_msgs__msg__Int8 retry_msg;
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist pwm_msg;
geometry_msgs__msg__Twist debug_msg;
geometry_msgs__msg__Twist velocity_msg;

// ? define publisher
rcl_publisher_t pub_debug;
rcl_publisher_t pub_odom;
rcl_publisher_t pub_pwm;
rcl_publisher_t pub_start;
rcl_publisher_t pub_team;
rcl_publisher_t pub_retry;

// ? define subscriber
rcl_subscription_t sub_velocity;

rcl_init_options_t init_options;

bool micro_ros_init_successful;

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

//------------------------------ < Main > -------------------------------------------//

void setup()
{
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  // motor1_encoder.attachSingleEdge(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B);
  // motor2_encoder.attachSingleEdge(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B);
  // motor3_encoder.attachSingleEdge(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B);
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(TEAM_BUTTON, INPUT_PULLUP);
  pinMode(RETRY_BUTTON, INPUT_PULLUP);
}

void loop()
{
  switch (state)
  {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    break;
  case AGENT_AVAILABLE:
    state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT)
    {
      destroy_entities();
    };
    break;
  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED)
    {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    break;
  case AGENT_DISCONNECTED:
    destroy_entities();
    state = WAITING_AGENT;
    break;
  default:
    break;
  }

  if (state == AGENT_CONNECTED)
  {
  }
  else
  {
    renew();
  }
}

//------------------------------ < Fuction > ----------------------------------------//
int lim_switch(int lim_pin)
{
  return !digitalRead(lim_pin);
}

void moveBase()
{
  if (((millis() - prev_velocity_time) >= 200))
  {
    velocity_msg.linear.x = 0.0;
    velocity_msg.linear.y = 0.0;
    velocity_msg.angular.z = 0.0;
  }

  Kinematics::rpm req_rpm = kinematics.getRPM(
      velocity_msg.linear.x,
      velocity_msg.linear.y,
      velocity_msg.angular.z);

  float current_rpm1 = motor1_encoder.getRPM();
  float current_rpm2 = motor2_encoder.getRPM();
  float current_rpm3 = motor3_encoder.getRPM();
  debug_msg.linear.x = current_rpm1;
  debug_msg.linear.y = current_rpm2;
  debug_msg.linear.z = current_rpm3;
  debug_msg.angular.x = req_rpm.motor1;
  debug_msg.angular.y = req_rpm.motor2;
  debug_msg.angular.z = req_rpm.motor3;

  Kinematics::pwm motor_pwm = kinematics.getPWM(motor1_pid.compute(req_rpm.motor1, current_rpm1), motor2_pid.compute(req_rpm.motor2, current_rpm2), motor3_pid.compute(req_rpm.motor3, current_rpm3));
  pwm_msg.linear.x = motor_pwm.motor1;
  pwm_msg.linear.y = motor_pwm.motor2;
  pwm_msg.linear.z = motor_pwm.motor3;
  pwm_msg.angular.x = motor_pwm.motor4;

  Kinematics::velocities current_vel = kinematics.getVelocities(
      current_rpm1,
      current_rpm2,
      current_rpm3);

  unsigned long now = millis();
  float vel_dt = (now - prev_odom_update) / 1000.0;
  prev_odom_update = now;
  odometry.update(
      vel_dt,
      current_vel.linear_x,
      current_vel.linear_y,
      current_vel.angular_z);
}

void syncTime()
{
  // get the current time from the agent
  unsigned long now = millis();
  RCSOFTCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  // now we can find the difference between ROS time and uC time
  time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
  struct timespec tp = {0};
  // add time difference between uC time and ROS time to
  // synchronize time with ROS
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;

  return tp;
}

void publishData()
{

  odom_msg = odometry.getData();

  if (isPowered)
  {
    start_msg.data = lim_switch(START_BUTTON);
    team_msg.data = lim_switch(TEAM_BUTTON);
    retry_msg.data = lim_switch(RETRY_BUTTON);
  }
  if (pre_power != lim_switch(START_BUTTON))
  {
    if (lim_switch(START_BUTTON) == 0)
    {
      isPowered = true;
    }
    pre_power = lim_switch(START_BUTTON);
  }
  struct timespec time_stamp = getTime();

  odom_msg.header.stamp.sec = time_stamp.tv_sec;
  odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

  RCSOFTCHECK(rcl_publish(&pub_pwm, &pwm_msg, NULL));
  RCSOFTCHECK(rcl_publish(&pub_team, &team_msg, NULL));
  RCSOFTCHECK(rcl_publish(&pub_odom, &odom_msg, NULL));
  RCSOFTCHECK(rcl_publish(&pub_start, &start_msg, NULL));
  RCSOFTCHECK(rcl_publish(&pub_debug, &debug_msg, NULL));
  RCSOFTCHECK(rcl_publish(&pub_retry, &retry_msg, NULL));
}

//------------------------------ < Ros Fuction > ------------------------------------//
bool create_entities()
{
  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 10);

  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  // create node
  RCCHECK(rclc_node_init_default(&node, "int32_publisher_rclc", "", &support));

  // TODO: create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

  // TODO: create publisher
  RCCHECK(rclc_publisher_init_best_effort(
      &pub_debug,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "debug/drive/input"));
  RCCHECK(rclc_publisher_init_default(
      &pub_pwm,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "drive/pwm"));
  RCCHECK(rclc_publisher_init_default(
      &pub_odom,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "odom/unfiltered"));
  RCCHECK(rclc_publisher_init_default(
      &pub_start,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
      "button/start"));
  RCCHECK(rclc_publisher_init_default(
      &pub_team,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
      "button/team"));
  RCCHECK(rclc_publisher_init_default(
      &pub_retry,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
      "button/retry"));

  // TODO: create subscriber
  RCCHECK(rclc_subscription_init_default(
      &sub_velocity,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));

  // TODO: create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_velocity, &velocity_msg, &sub_velocity_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  return true;
}

void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&pub_pwm, &node);
  rcl_publisher_fini(&pub_team, &node);
  rcl_publisher_fini(&pub_odom, &node);
  rcl_publisher_fini(&pub_start, &node);
  rcl_publisher_fini(&pub_retry, &node);
  rcl_publisher_fini(&pub_debug, &node);
  rcl_subscription_fini(&sub_velocity, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void renew()
{
}

//------------------------------ < Publisher Fuction > ------------------------------//

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)last_call_time;
  if (timer != NULL)
  {
    moveBase();
    publishData();
  }
}

//------------------------------ < Subscriber Fuction > -----------------------------//

void sub_velocity_callback(const void *msgin)
{
  prev_velocity_time = millis();
}
