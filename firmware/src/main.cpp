#include <micro_ros_platformio.h>
#include <Arduino.h>
#include <stdio.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

#include <config_flag.h>
#include <gripper.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/pose2_d.h>
#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/twist.h>

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      return false;              \
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
static uint32_t preT = 0;
bool preTS = false;

float tick1 = 0.0;
float tick2 = 0.0;
float tick3 = 0.0;
float speed1 = 0.0;
float speed2 = 0.0;
float speed3 = 0.0;

long positions[3];

AccelStepper step1(AccelStepper::DRIVER, STEP1_PWM, STEP1_DIR);
AccelStepper step2(AccelStepper::DRIVER, STEP2_PWM, STEP2_DIR);
AccelStepper step3(AccelStepper::DRIVER, STEP3_PWM, STEP3_DIR);
MultiStepper multiStep;

//------------------------------ < Fuction > ----------------------------------------//
int lim_switch(int lim_pin)
{
  return not digitalRead(lim_pin);
}

//------------------------------ < Ros Define > -------------------------------------//
// basic
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rcl_allocator_t allocator;
rclc_executor_t executor;

// ? define msg
geometry_msgs__msg__Pose2D position_msg;
geometry_msgs__msg__Vector3 speed_msg;
geometry_msgs__msg__Twist debug_msg;

// ? define publisher
rcl_publisher_t pub_debug;

// ? define subscriber
rcl_subscription_t sub_position;
rcl_subscription_t sub_speed;

rcl_init_options_t init_options;

bool micro_ros_init_successful;

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

//------------------------------ < Publisher Fuction > ------------------------------//

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)last_call_time;
  if (timer != NULL)
  {
    debug_msg.linear.x = tick1;
    debug_msg.linear.y = tick2;
    debug_msg.linear.z = tick3;
    debug_msg.angular.x = speed1;
    debug_msg.angular.y = speed2;
    debug_msg.angular.z = speed3;
    rcl_publish(&pub_debug, &debug_msg, NULL);
  }
}

//------------------------------ < Subscriber Fuction > -----------------------------//
void sub_position_callback(const void *msgin)
{
  const geometry_msgs__msg__Pose2D *position_msg = (const geometry_msgs__msg__Pose2D *)msgin;
  positions[0] = position_msg->x;
  positions[1] = position_msg->y;
  positions[2] = position_msg->theta;
  multiStep.moveTo(positions);
  multiStep.runSpeedToPosition();
  tick1 = step1.currentPosition();
  tick2 = step2.currentPosition();
  tick3 = step3.currentPosition();
}

void sub_speed_callback(const void *msgin)
{
  const geometry_msgs__msg__Vector3 *speed_msg = (const geometry_msgs__msg__Vector3 *)msgin;
  step1.setSpeed(speed_msg->x);
  step2.setSpeed(speed_msg->y);
  step3.setSpeed(speed_msg->z);
  speed1 = step1.speed();
  speed2 = step2.speed();
  speed3 = step3.speed();
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
      "debug/flag"));

  // TODO: create subscriber
  RCCHECK(rclc_subscription_init_default(
      &sub_position,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D),
      "gripper/flag/pos"));
  RCCHECK(rclc_subscription_init_default(
      &sub_speed,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
      "gripper/flag/speed"));

  // TODO: create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_position, &position_msg, &sub_position_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_speed, &speed_msg, &sub_speed_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  return true;
}

void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&pub_debug, &node);
  rcl_subscription_fini(&sub_position, &node);
  rcl_subscription_fini(&sub_speed, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void renew()
{
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
}

void setup()
{
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  step1.setMaxSpeed(2e4);
  step2.setMaxSpeed(2e4);
  step3.setMaxSpeed(2e4);

  multiStep.addStepper(step1);
  multiStep.addStepper(step2);
  multiStep.addStepper(step3);
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
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, LOW);
  }
  else
  {
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, HIGH);
    renew();
  }
}