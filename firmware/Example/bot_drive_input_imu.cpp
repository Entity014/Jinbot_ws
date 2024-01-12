#include <micro_ros_platformio.h>
#include <Arduino.h>
#include <stdio.h>

#include <config_drive_input.h>
#include <imu.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>

#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            return false;            \
        }                            \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
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
unsigned long long time_offset = 0;
IMU imu;

//------------------------------ < Fuction Prototype > ------------------------------//
void syncTime();
void publishData();
struct timespec getTime();
//------------------------------ < Ros Fuction Prototype > --------------------------//

void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
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
sensor_msgs__msg__Imu imu_msg;

// ? define publisher
rcl_publisher_t pub_imu;

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
    imu.init();
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

    imu_msg = imu.getData();

    struct timespec time_stamp = getTime();

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&pub_imu, &imu_msg, NULL));
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
    RCCHECK(rclc_publisher_init_default(
        &pub_imu,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"));

    // TODO: create executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    return true;
}

void destroy_entities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&pub_imu, &node);
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
        publishData();
    }
}