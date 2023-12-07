#include <micro_ros_platformio.h>
#include <Arduino.h>
#include <stdio.h>
#include <ESP32_Servo.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

#include <config_flag.h>
#include <gripper.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

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

long positions[3];
long pre_positions[3];

Servo servo1;
Servo servo2;

AccelStepper stepM1(AccelStepper::DRIVER, STEP1_PWM, STEP1_DIR);
AccelStepper stepM2(AccelStepper::DRIVER, STEP2_PWM, STEP2_DIR);
AccelStepper stepM3(AccelStepper::DRIVER, STEP3_PWM, STEP3_DIR);
MultiStepper multiStep;

Parallel_3dof::angular angular;
Parallel_3dof flagGripper(LENGTH_A, LENGTH_B, LENGTH_C, LENGTH_D, LENGTH_E, LENGTH_F);

//------------------------------ < Fuction Prototype > ------------------------------//

int lim_switch(int lim_pin);
void task_arduino_fcn(void *arg);

//------------------------------ < Ros Fuction Prototype > --------------------------//

void task_ros_fcn(void *arg);
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void sub_position_callback(const void *msgin);
void sub_speed_callback(const void *msgin);
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

//------------------------------ < Main > -------------------------------------------//

void setup()
{
  xTaskCreate(
      task_arduino_fcn, /* Task function. */
      "Arduino Task",   /* String with name of task. */
      1024,             /* Stack size in bytes. */
      NULL,             /* Parameter passed as input of the task */
      0,                /* Priority of the task. */
      NULL);            /* Task handle. */

  xTaskCreate(
      task_ros_fcn, /* Task function. */
      "Ros Task",   /* String with name of task. */
      4096,         /* Stack size in bytes. */
      NULL,         /* Parameter passed as input of the task */
      1,            /* Priority of the task. */
      NULL);        /* Task handle. */
}

void loop()
{
}

//------------------------------ < Fuction > ----------------------------------------//

int lim_switch(int lim_pin)
{
  return !digitalRead(lim_pin);
}

void task_arduino_fcn(void *arg)
{
  pinMode(LIMIT1, INPUT_PULLUP);
  pinMode(LIMIT2, INPUT_PULLUP);
  pinMode(LIMIT3, INPUT_PULLUP);

  stepM1.setMaxSpeed(STEP1_SPEED);
  stepM2.setMaxSpeed(STEP2_SPEED);
  stepM3.setMaxSpeed(STEP3_SPEED);
  stepM1.setAcceleration(STEP1_SPEED);
  stepM2.setAcceleration(STEP2_SPEED);
  stepM3.setAcceleration(STEP3_SPEED);

  multiStep.addStepper(stepM1);
  multiStep.addStepper(stepM2);
  multiStep.addStepper(stepM3);

  servo1.attach(SERVO1);
  servo2.attach(SERVO2);
  servo1.write(0);
  servo2.write(90);
  positions[0] = 1e6;
  positions[1] = 1e6;
  positions[2] = 1e6;
  multiStep.moveTo(positions);
  while (true)
  {
    if (lim_switch(LIMIT1))
    {
      stepM1.setSpeed(0);
      stepM1.setCurrentPosition(0);
    }
    if (lim_switch(LIMIT2))
    {
      stepM2.setSpeed(0);
      stepM2.setCurrentPosition(0);
    }
    if (lim_switch(LIMIT3))
    {
      stepM3.setSpeed(0);
      stepM3.setCurrentPosition(0);
    }
    if ((pre_positions[0] != positions[0]) || (pre_positions[1] != positions[1]) || (pre_positions[2] != positions[2]))
    {
      pre_positions[0] = positions[0];
      pre_positions[1] = positions[1];
      pre_positions[2] = positions[2];
      multiStep.moveTo(positions);
    }
    multiStep.run();
  }
}

//------------------------------ < Ros Fuction > ------------------------------------//

void task_ros_fcn(void *arg)
{
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  while (true)
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
      digitalWrite(LED2_PIN, HIGH);
    }
    else
    {
      digitalWrite(LED1_PIN, LOW);
      digitalWrite(LED2_PIN, HIGH);
      renew();
    }
  }
}

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
  servo1.write(0);
  servo2.write(90);
}

//------------------------------ < Publisher Fuction > ------------------------------//

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)last_call_time;
  if (timer != NULL)
  {
    debug_msg.linear.x = stepM1.currentPosition();
    debug_msg.linear.y = stepM2.currentPosition();
    debug_msg.linear.z = stepM3.currentPosition();
    debug_msg.angular.x = stepM1.speed();
    debug_msg.angular.y = stepM2.speed();
    debug_msg.angular.z = stepM3.speed();
    rcl_publish(&pub_debug, &debug_msg, NULL);
  }
}

//------------------------------ < Subscriber Fuction > -----------------------------//

void sub_position_callback(const void *msgin)
{
  const geometry_msgs__msg__Pose2D *position_msg = (const geometry_msgs__msg__Pose2D *)msgin;
  angular = flagGripper.getAngular(position_msg->x, position_msg->y);
  if ((position_msg->x != 0) || (position_msg->y != 0))
  {
    positions[0] = ((float)177000 / 90) * angular.angular_a;
    positions[1] = ((float)177000 / 90) * angular.angular_b;
    // positions[2] = position_msg->theta;
  }
  // positions[0] = position_msg->x;
  // positions[1] = position_msg->y;
}

void sub_speed_callback(const void *msgin)
{
  const geometry_msgs__msg__Vector3 *speed_msg = (const geometry_msgs__msg__Vector3 *)msgin;
}