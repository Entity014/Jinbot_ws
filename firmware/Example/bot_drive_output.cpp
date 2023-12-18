#include <micro_ros_platformio.h>
#include <Arduino.h>
#include <stdio.h>

#include <config_drive_output.h>
#include <motor.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/imu.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            return false;            \
        }                            \
    }
#define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS)      \
        {                                  \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)

//------------------------------ < ESP32 Define > -----------------------------------//

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

//------------------------------ < Fuction Prototype > ------------------------------//

void task_arduino_fcn(void *arg);

//------------------------------ < Ros Fuction Prototype > --------------------------//

void task_ros_fcn(void *arg);
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void sub_pwm_callback(const void *msgin);
void sub_main_callback(const void *msgin);
void sub_team_callback(const void *msgin);
void sub_retry_callback(const void *msgin);
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
std_msgs__msg__String main_msg;
std_msgs__msg__String team_msg;
std_msgs__msg__String retry_msg;
geometry_msgs__msg__Twist debug_msg;
geometry_msgs__msg__Twist pwm_msg;

// ? define publisher
rcl_publisher_t pub_debug;

// ? define subscriber
rcl_subscription_t sub_pwm;
rcl_subscription_t sub_main;
rcl_subscription_t sub_team;
rcl_subscription_t sub_retry;

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

void task_arduino_fcn(void *arg)
{
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(RLED1_PIN, OUTPUT);
    pinMode(GLED1_PIN, OUTPUT);
    pinMode(BLED1_PIN, OUTPUT);
    pinMode(RLED2_PIN, OUTPUT);
    pinMode(GLED2_PIN, OUTPUT);
    pinMode(BLED2_PIN, OUTPUT);
    while (true)
    {
        if (main_msg.data.data == "Reset")
        {
            analogWrite(RLED1_PIN, 1023);
            analogWrite(GLED1_PIN, 1023);
            analogWrite(BLED1_PIN, 0);
            analogWrite(RLED2_PIN, 1023);
            analogWrite(GLED2_PIN, 1023);
            analogWrite(BLED2_PIN, 0);
        }
        else
        {
            if (team_msg.data.data == "Blue")
            {
                analogWrite(RLED1_PIN, 0);
                analogWrite(GLED1_PIN, 0);
                analogWrite(BLED1_PIN, 1023);
            }
            else if (team_msg.data.data == "Red")
            {
                analogWrite(RLED1_PIN, 1023);
                analogWrite(GLED1_PIN, 0);
                analogWrite(BLED1_PIN, 0);
            }

            if (retry_msg.data.data == "First")
            {
                analogWrite(RLED2_PIN, 0);
                analogWrite(GLED2_PIN, 0);
                analogWrite(BLED2_PIN, 1023);
            }
            else if (retry_msg.data.data == "Second")
            {
                analogWrite(RLED2_PIN, 1023);
                analogWrite(GLED2_PIN, 0);
                analogWrite(BLED2_PIN, 0);
            }
            else
            {
                analogWrite(RLED2_PIN, 0);
                analogWrite(GLED2_PIN, 0);
                analogWrite(BLED2_PIN, 0);
            }
        }
    }
}

//------------------------------ < Ros Fuction > ------------------------------------//

void task_ros_fcn(void *arg)
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

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
            digitalWrite(LED2_PIN, LOW);
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
        "debug/drive/output"));

    // TODO: create subscriber
    RCCHECK(rclc_subscription_init_default(
        &sub_pwm,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "drive/pwm"));
    RCCHECK(rclc_subscription_init_default(
        &sub_main,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "state/main"));
    RCCHECK(rclc_subscription_init_default(
        &sub_team,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "state/team"));
    RCCHECK(rclc_subscription_init_default(
        &sub_retry,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "state/retry"));

    // TODO: create executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_pwm, &pwm_msg, &sub_pwm_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_main, &main_msg, &sub_main_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_team, &team_msg, &sub_team_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_retry, &retry_msg, &sub_retry_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    return true;
}

void destroy_entities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&pub_debug, &node);
    rcl_subscription_fini(&sub_pwm, &node);
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
        debug_msg.angular.y = 0.0;
        debug_msg.angular.z = 0.0;
        rcl_publish(&pub_debug, &debug_msg, NULL);
    }
}

//------------------------------ < Subscriber Fuction > -----------------------------//
void sub_pwm_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *pwm_msg = (const geometry_msgs__msg__Twist *)msgin;
    debug_msg.linear.x = pwm_msg->linear.x;
    debug_msg.linear.y = pwm_msg->linear.y;
    debug_msg.linear.z = pwm_msg->linear.z;
    debug_msg.angular.x = pwm_msg->angular.x;
    motor1_controller.spin((int)pwm_msg->linear.x);
    motor2_controller.spin((int)pwm_msg->linear.y);
    motor3_controller.spin((int)pwm_msg->linear.z);
    motor4_controller.spin((int)pwm_msg->angular.x);
}

void sub_main_callback(const void *msgin)
{
    const std_msgs__msg__String *main_msg = (const std_msgs__msg__String *)msgin;
}

void sub_team_callback(const void *msgin)
{
    const std_msgs__msg__String *team_msg = (const std_msgs__msg__String *)msgin;
}

void sub_retry_callback(const void *msgin)
{
    const std_msgs__msg__String *retry_msg = (const std_msgs__msg__String *)msgin;
}