/**
 * @file mbot_classic_ros.c
 * @brief MicroROS integration for MBot Classic
 */
#include <math.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/time_sync.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <pico/multicore.h>
#include "pico/time.h"
#include <hardware/clocks.h>
#include <hardware/adc.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>

// mbotlib
#include <mbot/motor/motor.h>
#include <mbot/encoder/encoder.h>
#include <mbot/fram/fram.h>
#include <mbot/imu/imu.h>
#include <mbot/utils/utils.h>

// mbot_classic_ros
#include "mbot_classic_ros.h"
#include "mbot_odometry.h"
#include "mbot_ros_comms.h"
#include "mbot_print.h"

// comms
#include <comms/pico_uart_transports.h>
#include <comms/dual_cdc.h>

// robot control
#include <rc/math/filter.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// Global state variables
mbot_state_t mbot_state = {0};
mbot_cmd_t mbot_cmd = {0};
mbot_params_t params;
mbot_bhy_config_t mbot_imu_config;
mbot_bhy_data_t mbot_imu_data;
static bool enable_pwm_lpf = true;
rc_filter_t mbot_left_pwm_lpf;
rc_filter_t mbot_right_pwm_lpf;

// Global MicroROS objects
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;

// Timer for periodic publishing
static rcl_timer_t ros_publish_timer;
static repeating_timer_t mbot_loop_timer;

int mbot_init_micro_ros(void);
int mbot_spin_micro_ros(void);
static void mbot_publish_state(void);
static bool mbot_loop(repeating_timer_t *rt);
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);

// helper functions
static int mbot_init_hardware(void);
static void mbot_read_imu(void);
static void mbot_read_encoders(void);
static void mbot_read_adc(void);
static float calibrated_pwm_from_vel_cmd(float vel_cmd, int motor_idx);
static void mbot_calculate_motor_vel(void);
static void mbot_calculate_diff_body_vel(float wheel_left_vel, float wheel_right_vel, float* vx, float* vy, float* wz);
static void print_mbot_params(const mbot_params_t* params);

// Thread-safe helpers for mbot_state and mbot_cmd
static void get_mbot_state_safe(mbot_state_t* dest);
static void set_mbot_state_safe(const mbot_state_t* src);
static void get_mbot_cmd_safe(mbot_cmd_t* dest);
static void set_mbot_cmd_safe(const mbot_cmd_t* src);

// Initialize microROS
int mbot_init_micro_ros(void) {
    allocator = rcl_get_default_allocator();

    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        printf("rclc_support_init failed: %d\n", ret);
        return MBOT_ERROR;
    }

    // Synchronize time with agent
    rmw_ret_t sync_ret = rmw_uros_sync_session(2000);
    if (sync_ret != RMW_RET_OK) {
        printf("[FATAL] Time sync with agent failed: %d\n", sync_ret);
        while(1) { tight_loop_contents(); }
    }
        
    ret = rclc_node_init_default(&node, "microros_node", "", &support);
    if (ret != RCL_RET_OK) {
        printf("rclc_node_init_default failed: %d\n", ret);
        rclc_support_fini(&support);
        return MBOT_ERROR;
    }
    
    ret = mbot_ros_comms_init_messages(&allocator);
    if (ret != MBOT_OK) return MBOT_ERROR;

    ret = mbot_ros_comms_init_publishers(&node);
    if (ret != MBOT_OK) return MBOT_ERROR;

    ret = mbot_ros_comms_init_subscribers(&node);
    if (ret != MBOT_OK) return MBOT_ERROR;

    ret = rclc_timer_init_default(
        &ros_publish_timer,
        &support,
        RCL_MS_TO_NS((int)(ROS_TIMER_PERIOD * 1000)),
        timer_callback);
    if (ret != RCL_RET_OK) {
        printf("[ERROR] Timer init failed: %d\n", ret);
        return MBOT_ERROR;
    }
 
    ret = rclc_executor_init(&executor, &support.context, 5, &allocator);
    if (ret != RCL_RET_OK) {
        return MBOT_ERROR;
    }
    
    ret = rclc_executor_add_timer(&executor, &ros_publish_timer);
    if (ret != RCL_RET_OK) {
        printf("[ERROR] Adding timer to executor failed: %d\n", ret);
        return MBOT_ERROR;
    }
    
    ret = rcl_timer_call(&ros_publish_timer);
    if (ret != RCL_RET_OK) {
        printf("[ERROR] Timer call failed: %d\n", ret);
    }

    ret = mbot_ros_comms_add_to_executor(&executor);
    if (ret != MBOT_OK) return MBOT_ERROR;
    
    return MBOT_OK;
}   

// Handle incoming ROS messages
int mbot_spin_micro_ros(void) {
    rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
        printf("microROS spin error: %d\r\n", ret);
        return MBOT_ERROR;
    }
    
    return MBOT_OK;
}

// Publish all robot state to ROS topics
static void mbot_publish_state(void) {
    rcl_ret_t ret;
    int64_t now = rmw_uros_epoch_nanos();
    if (!rmw_uros_epoch_synchronized()) {
        printf("[FATAL] Last time synchronization failed\n");
        while(1) { tight_loop_contents(); }
    }

    // Get state safely
    mbot_state_t local_state;
    get_mbot_state_safe(&local_state);
    
    // Publish IMU data
    imu_msg.header.stamp.sec = now / 1000000000;
    imu_msg.header.stamp.nanosec = now % 1000000000;
    imu_msg.angular_velocity.x = local_state.imu_gyro[0];
    imu_msg.angular_velocity.y = local_state.imu_gyro[1];
    imu_msg.angular_velocity.z = local_state.imu_gyro[2];
    imu_msg.linear_acceleration.x = local_state.imu_accel[0];
    imu_msg.linear_acceleration.y = local_state.imu_accel[1];
    imu_msg.linear_acceleration.z = local_state.imu_accel[2];
    imu_msg.orientation.w = local_state.imu_quat[0];
    imu_msg.orientation.x = local_state.imu_quat[1];
    imu_msg.orientation.y = local_state.imu_quat[2];
    imu_msg.orientation.z = local_state.imu_quat[3];
    ret = rcl_publish(&imu_publisher, &imu_msg, NULL);
    if (ret != RCL_RET_OK) {
        printf("Error publishing IMU message: %d\r\n", ret);
    }
    // Publish odometry data
    odom_msg.header.stamp.sec = now / 1000000000;
    odom_msg.header.stamp.nanosec = now % 1000000000;
    odom_msg.pose.pose.position.x = local_state.odom_x;
    odom_msg.pose.pose.position.y = local_state.odom_y;
    odom_msg.pose.pose.position.z = 0.0;
    float cy = cos(local_state.odom_theta * 0.5);
    float sy = sin(local_state.odom_theta * 0.5);
    odom_msg.pose.pose.orientation.w = cy;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sy;
    odom_msg.twist.twist.linear.x = local_state.vx;
    odom_msg.twist.twist.linear.y = local_state.vy;
    odom_msg.twist.twist.angular.z = local_state.wz;
    ret = rcl_publish(&odom_publisher, &odom_msg, NULL);
    if (ret != RCL_RET_OK) {
        printf("Error publishing odometry message: %d\r\n", ret);
    }
    // Publish odom->base_footprint TF as tf2_msgs/msg/TFMessage
    tf_msg.transforms.data[0].header.stamp.sec = now / 1000000000;
    tf_msg.transforms.data[0].header.stamp.nanosec = now % 1000000000;
    tf_msg.transforms.data[0].header.frame_id = odom_msg.header.frame_id;
    tf_msg.transforms.data[0].child_frame_id = odom_msg.child_frame_id;
    tf_msg.transforms.data[0].transform.translation.x = odom_msg.pose.pose.position.x;
    tf_msg.transforms.data[0].transform.translation.y = odom_msg.pose.pose.position.y;
    tf_msg.transforms.data[0].transform.translation.z = odom_msg.pose.pose.position.z;
    tf_msg.transforms.data[0].transform.rotation = odom_msg.pose.pose.orientation;
    tf_msg.transforms.size = 1;
    ret = rcl_publish(&tf_publisher, &tf_msg, NULL);
    if (ret != RCL_RET_OK) {
        printf("Error publishing TF message: %d\r\n", ret);
    }
    // Publish Mbot velocity data
    mbot_vel_msg.linear.x = local_state.vx;
    mbot_vel_msg.linear.y = local_state.vy;
    mbot_vel_msg.angular.z = local_state.wz;
    ret = rcl_publish(&mbot_vel_publisher, &mbot_vel_msg, NULL);
    if (ret != RCL_RET_OK) {
        printf("Error publishing mbot_vel message: %d\r\n", ret);
    }
    // Publish motor velocities
    motor_vel_msg.x = local_state.wheel_vel[MOT_L];
    motor_vel_msg.y = local_state.wheel_vel[MOT_R];
    motor_vel_msg.z = local_state.wheel_vel[MOT_UNUSED];
    ret = rcl_publish(&motor_vel_publisher, &motor_vel_msg, NULL);
    if (ret != RCL_RET_OK) {
        printf("Error publishing motor velocity message: %d\r\n", ret);
    }
}

// Main robot logic loop, runs at MAIN_LOOP_HZ (called by hardware timer)
static bool mbot_loop(repeating_timer_t *rt) {
    {   // Critical section for sensor and state update
        ENTER_CRITICAL();
        mbot_read_encoders();    
        mbot_read_imu();
        mbot_read_adc();
        mbot_calculate_motor_vel();
        mbot_calculate_diff_body_vel(
            mbot_state.wheel_vel[MOT_L],
            mbot_state.wheel_vel[MOT_R],
            &mbot_state.vx,
            &mbot_state.vy,
            &mbot_state.wz
        );
        mbot_calculate_odometry(
            mbot_state.vx,
            mbot_state.vy,
            mbot_state.wz,
            MAIN_LOOP_PERIOD,
            &mbot_state.odom_x,
            &mbot_state.odom_y,
            &mbot_state.odom_theta
        );
        int64_t now = time_us_64();
        mbot_state.timestamp_us = now;
        EXIT_CRITICAL();
    }
    // Get command safely
    mbot_cmd_t local_cmd;
    get_mbot_cmd_safe(&local_cmd);
    bool cmd_fresh = (time_us_64() - local_cmd.timestamp_us) < MBOT_TIMEOUT_US;
    float pwm_left = 0.0f, pwm_right = 0.0f;
    if (cmd_fresh) {
        switch (local_cmd.drive_mode) {
            case MODE_MOTOR_PWM:
                pwm_left = local_cmd.motor_pwm[MOT_L];
                pwm_right = local_cmd.motor_pwm[MOT_R];
                break;
            case MODE_MOTOR_VEL_OL:
                pwm_left = calibrated_pwm_from_vel_cmd(local_cmd.wheel_vel[MOT_L], MOT_L);
                pwm_right = calibrated_pwm_from_vel_cmd(local_cmd.wheel_vel[MOT_R], MOT_R);
                break;
            case MODE_MBOT_VEL: {
                float vel_left = (local_cmd.vx - DIFF_BASE_RADIUS * local_cmd.wz) / DIFF_WHEEL_RADIUS;
                float vel_right = (-local_cmd.vx - DIFF_BASE_RADIUS * local_cmd.wz) / DIFF_WHEEL_RADIUS;
                float vel_left_comp = params.motor_polarity[MOT_L] * vel_left;
                float vel_right_comp = params.motor_polarity[MOT_R] * vel_right;
                pwm_left = calibrated_pwm_from_vel_cmd(vel_left_comp, MOT_L);
                pwm_right = calibrated_pwm_from_vel_cmd(vel_right_comp, MOT_R);
                break;
            }
            default:
                pwm_left = 0.0f;
                pwm_right = 0.0f;
        }
    } else {
        pwm_left = 0.0f;
        pwm_right = 0.0f;
    }
    // Low-pass filter if enabled
    if (enable_pwm_lpf) {
        pwm_left = rc_filter_march(&mbot_left_pwm_lpf, pwm_left);
        pwm_right = rc_filter_march(&mbot_right_pwm_lpf, pwm_right);
    }
    // Set motors
    mbot_motor_set_duty(MOT_L, pwm_left);
    mbot_motor_set_duty(MOT_R, pwm_right);
    {   // Critical section for storing motor PWM to mbot_state
        ENTER_CRITICAL();
        mbot_state.motor_pwm[MOT_L] = pwm_left;
        mbot_state.motor_pwm[MOT_R] = pwm_right;
        EXIT_CRITICAL();
    }
    return true; 
}

// Timer callback for periodic ROS publishing (runs at ROS_TIMER_HZ)
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    mbot_publish_state();
}

/**
 * @brief Main entry point
 */
int main() {
    // Initialize Dual CDC and stdio
    stdio_init_all();
    dual_cdc_init();
    mbot_wait_ms(2000);
    
    printf("\r\nMBot Classic Firmware (ROS2)\r\n");
    printf("--------------------------------\r\n");

    mbot_init_hardware();

    mbot_read_fram(0, sizeof(params), (uint8_t*)&params);

    printf("\nCalibration Parameters:\n");
    print_mbot_params(&params);
    mbot_wait_ms(1000);

    int validate_status = validate_mbot_classic_FRAM_data(&params, MOT_L, MOT_R, MOT_UNUSED);
    if (validate_status < 0){
        printf("Failed to validate FRAM Data! Error code: %d\n", validate_status);
    }

    printf("\nStarting MBot Loop...\n");
    mbot_state.last_encoder_time = time_us_64(); 
    if (!add_repeating_timer_ms((int32_t)(MAIN_LOOP_PERIOD * 1000.0f), mbot_loop, NULL, &mbot_loop_timer)){
        printf("Failed to add control loop timer! Halting.\r\n");
        while(1) {tight_loop_contents();}
    }

    rmw_ret_t rmw_ret = rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );
    if (rmw_ret != RMW_RET_OK) {
        printf("rmw_uros_set_custom_transport failed: %d\n", rmw_ret);
        while(1) {tight_loop_contents();}
    }

    printf("Pinging Agent...\n");
    // Wait for agent successful ping for 3 minutes.
    const int timeout_ms = 1000;
    const uint8_t attempts = 180;
    rcl_ret_t ping_ret = rmw_uros_ping_agent(timeout_ms, attempts);
    if (ping_ret != RMW_RET_OK) {
        printf("[FATAL] Failed to ping Micro-ROS Agent!\n");
        printf("[FATAL] Please re-start the agent and press the reset button.\n");
        while(1) {tight_loop_contents();}
    }

    printf("Initializing Micro-ROS...\n");
    if (mbot_init_micro_ros() != MBOT_OK) {
        printf("[FATAL] Micro-ROS initialization failed\n");
        while(1) {tight_loop_contents();}
    }

    printf("Done Booting Up!\n");
    fflush(stdout); 

    // Main loop: if a fatal error occurs, halt and wait for reset
    static int64_t last_200ms_time = 0;
    while (1) {
        dual_cdc_task(); // Keep USB alive

        if (mbot_spin_micro_ros() != MBOT_OK) {
            printf("[FATAL] Micro-ROS spin failed. Please check agent connection and press the reset button.\n");
            while(1) { tight_loop_contents(); }
        }

        if (time_us_64() - last_200ms_time > 200000) { // 200ms interval
            last_200ms_time = time_us_64();
            mbot_print_state(&mbot_state);
        }
        sleep_ms(10); // Small delay
    }
    return 0;
}

/******************************************************
 * Helper Functions
 * ----------------------------------------------------
 * These functions are used internally by the main control functions.
 * They are not intended for modification by students. These functions
 * provide lower-level control and utility support.
 ******************************************************/
static int mbot_init_hardware(void){
    printf("Initializing Hardwares...\n");
    // Initialize Motors
    mbot_motor_init(MOT_L);
    mbot_motor_init(MOT_R);
    mbot_encoder_init();

    // Initialize the IMU 
    mbot_imu_config = mbot_imu_default_config();
    mbot_imu_config.sample_rate = 200;
    mbot_imu_init(&mbot_imu_data, mbot_imu_config);

    // Initialize ADC
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    adc_gpio_init(29);

    // Initialize PWM LPFs for smoother motion
    mbot_left_pwm_lpf = rc_filter_empty();
    mbot_right_pwm_lpf = rc_filter_empty();
    rc_filter_first_order_lowpass(&mbot_left_pwm_lpf, MAIN_LOOP_PERIOD, 4.0 * MAIN_LOOP_PERIOD);
    rc_filter_first_order_lowpass(&mbot_right_pwm_lpf, MAIN_LOOP_PERIOD, 4.0 * MAIN_LOOP_PERIOD);

    // Initialize FRAM
    mbot_init_fram();
    return MBOT_OK;
}

static void mbot_read_imu(void) {
    for(int i = 0; i < 3; i++) {
        mbot_state.imu_gyro[i] = mbot_imu_data.gyro[i];
        mbot_state.imu_accel[i] = mbot_imu_data.accel[i];
        mbot_state.imu_mag[i] = mbot_imu_data.mag[i];
        mbot_state.imu_rpy[i] = mbot_imu_data.rpy[i];
    }
    for(int i = 0; i < 4; i++) {
        mbot_state.imu_quat[i] = mbot_imu_data.quat[i];
    }
}

static void mbot_read_encoders(void) {
    int64_t now = time_us_64();

    // Calculate actual delta time since last encoder read
    // mbot_state.last_encoder_time is initialized in main() before the loop starts
    mbot_state.encoder_delta_t = now - mbot_state.last_encoder_time;
    
    // If dt is zero or negative (e.g. time_us_64 wraps or error), use nominal period
    if (mbot_state.encoder_delta_t <= 0) {
        mbot_state.encoder_delta_t = ((int64_t)(MAIN_LOOP_PERIOD * 1000000.0f));
    }

    mbot_state.last_encoder_time = now; // Update for the next cycle
    mbot_state.encoder_ticks[MOT_L] = mbot_encoder_read_count(MOT_L);
    mbot_state.encoder_ticks[MOT_R] = mbot_encoder_read_count(MOT_R);
    mbot_state.encoder_delta_ticks[MOT_L] = mbot_encoder_read_delta(MOT_L);
    mbot_state.encoder_delta_ticks[MOT_R] = mbot_encoder_read_delta(MOT_R);
}

static void mbot_read_adc(void) {
    const float conversion_factor = 3.0f / (1 << 12);
    int16_t raw;
    for(int i = 0; i < 4; i++) {
        adc_select_input(i);
        raw = adc_read();
        mbot_state.analog_in[i] = conversion_factor * raw;
    }
    // last channel is battery voltage (has 5x divider)
    mbot_state.analog_in[3] = 5.0f * conversion_factor * raw;
}

static float calibrated_pwm_from_vel_cmd(float vel_cmd, int motor_idx) {
    if (vel_cmd > 0.0f) {
        return (vel_cmd * params.slope_pos[motor_idx]) + params.itrcpt_pos[motor_idx];
    } else if (vel_cmd < 0.0f) {
        return (vel_cmd * params.slope_neg[motor_idx]) + params.itrcpt_neg[motor_idx];
    }
    return 0.0f;
}

static void mbot_calculate_motor_vel(void) {
    float conversion = (1.0f / GEAR_RATIO) * (1.0f / ENCODER_RES) * 1E6f * 2.0f * PI;
    int64_t delta_t = mbot_state.encoder_delta_t; // Use the corrected delta_time
    
    if (delta_t <= 0) { /* Avoid division by zero or invalid dt */ 
        mbot_state.wheel_vel[MOT_L] = 0.0f;
        mbot_state.wheel_vel[MOT_R] = 0.0f;
        return; 
    }

    mbot_state.wheel_vel[MOT_L] = params.encoder_polarity[MOT_L] * 
        (conversion / delta_t) * mbot_state.encoder_delta_ticks[MOT_L];
    mbot_state.wheel_vel[MOT_R] = params.encoder_polarity[MOT_R] * 
        (conversion / delta_t) * mbot_state.encoder_delta_ticks[MOT_R];
}

static void mbot_calculate_diff_body_vel(float wheel_left_vel, float wheel_right_vel, float* vx, float* vy, float* wz) {
    // Calculate forward velocity and angular velocity
    *vx = DIFF_WHEEL_RADIUS * (wheel_left_vel - wheel_right_vel) / 2.0f;
    *vy = 0.0;
    *wz = DIFF_WHEEL_RADIUS * (-wheel_left_vel - wheel_right_vel) / (2.0f * DIFF_BASE_RADIUS);
}

static void print_mbot_params(const mbot_params_t* params) {
    printf("Motor Polarity: %d %d\n", params->motor_polarity[MOT_L], params->motor_polarity[MOT_R]);
    printf("Encoder Polarity: %d %d\n", params->encoder_polarity[MOT_L], params->encoder_polarity[MOT_R]);
    printf("Positive Slope: %f %f\n", params->slope_pos[MOT_L], params->slope_pos[MOT_R]);
    printf("Positive Intercept: %f %f\n", params->itrcpt_pos[MOT_L], params->itrcpt_pos[MOT_R]);
    printf("Negative Slope: %f %f\n", params->slope_neg[MOT_L], params->slope_neg[MOT_R]);
    printf("Negative Intercept: %f %f\n", params->itrcpt_neg[MOT_L], params->itrcpt_neg[MOT_R]);
}

static void get_mbot_state_safe(mbot_state_t* dest) {
    ENTER_CRITICAL();
    *dest = mbot_state;
    EXIT_CRITICAL();
}
static void set_mbot_state_safe(const mbot_state_t* src) {
    ENTER_CRITICAL();
    mbot_state = *src;
    EXIT_CRITICAL();
}
static void get_mbot_cmd_safe(mbot_cmd_t* dest) {
    ENTER_CRITICAL();
    *dest = mbot_cmd;
    EXIT_CRITICAL();
}
static void set_mbot_cmd_safe(const mbot_cmd_t* src) {
    ENTER_CRITICAL();
    mbot_cmd = *src;
    EXIT_CRITICAL();
}