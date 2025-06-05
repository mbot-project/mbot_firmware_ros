#ifndef MBOT_CONTROLLER_H
#define MBOT_CONTROLLER_H

#include <stdbool.h>
#include <rclc_parameter/rclc_parameter.h>

typedef struct {
    float kp;
    float ki;
    float kd;
} pid_params_t;

typedef struct {
    pid_params_t left_wheel;
    pid_params_t right_wheel;
    pid_params_t body_vel_vx;
    pid_params_t body_vel_wz;
} mbot_pid_config_t;

// Parameter server
int init_parameter_server(rclc_parameter_server_t* parameter_server, rcl_node_t* node);
bool parameter_callback(const Parameter * old_param, const Parameter * new_param, void * context);

// Init
int mbot_controller_init(void);

// Motor velocity controller - returns PWM values for left and right motors
void mbot_motor_vel_controller(float target_left_vel, float target_right_vel, 
                              float current_left_vel, float current_right_vel,
                              float* left_pwm_out, float* right_pwm_out);

// Body velocity controller - returns PWM values for left and right motors
void mbot_body_vel_controller(float target_vx, float target_wz,
                             float current_vx, float current_wz,
                             float* vx_pwm, float* wz_pwm);

// PID gains global config
extern mbot_pid_config_t pid_gains;

#endif