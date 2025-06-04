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
int mbot_controller_init(mbot_pid_config_t* pid_gains);

// Motor velocity controller
int mbot_motor_vel_controller(void);

// Body velocity controller
int mbot_body_vel_controller(void);

// PID gains global config
extern mbot_pid_config_t pid_gains;

#endif