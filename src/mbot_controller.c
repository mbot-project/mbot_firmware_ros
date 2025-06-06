#include "mbot_controller.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <mbot/defs/mbot_params.h>
#include "config/mbot_classic_config.h"
#include <rc/math/filter.h>
#include <stdio.h>
#include <string.h>

#define DEFAULT_TF (MAIN_LOOP_PERIOD)  // Tf must be > dt/2 for stability

// Global PID controller variables
mbot_pid_config_t pid_gains = {
    .left_wheel = { .kp = 0.0, .ki = 0.0, .kd = 0.0, .tf = DEFAULT_TF },
    .right_wheel = { .kp = 0.0, .ki = 0.0, .kd = 0.0, .tf = DEFAULT_TF },
    .body_vel_vx = { .kp = 0.0, .ki = 0.0, .kd = 0.0, .tf = DEFAULT_TF },
    .body_vel_wz = { .kp = 0.0, .ki = 0.0, .kd = 0.0, .tf = DEFAULT_TF },
};
static bool pid_updated = false;

// PID filters
rc_filter_t left_wheel_pid;
rc_filter_t right_wheel_pid;
rc_filter_t body_vel_vx_pid;
rc_filter_t body_vel_wz_pid;

int mbot_controller_init(void) {
    // Initialize PID controllers
    left_wheel_pid = rc_filter_empty();
    right_wheel_pid = rc_filter_empty();
    body_vel_vx_pid = rc_filter_empty();
    body_vel_wz_pid = rc_filter_empty();
    
    // Set up PID controllers with the provided gains
    rc_filter_pid(&left_wheel_pid, 
                  pid_gains.left_wheel.kp, 
                  pid_gains.left_wheel.ki, 
                  pid_gains.left_wheel.kd, 
                  pid_gains.left_wheel.tf, 
                  MAIN_LOOP_PERIOD);
                  
    rc_filter_pid(&right_wheel_pid, 
                  pid_gains.right_wheel.kp, 
                  pid_gains.right_wheel.ki, 
                  pid_gains.right_wheel.kd, 
                  pid_gains.right_wheel.tf, 
                  MAIN_LOOP_PERIOD);
                  
    rc_filter_pid(&body_vel_vx_pid, 
                  pid_gains.body_vel_vx.kp, 
                  pid_gains.body_vel_vx.ki, 
                  pid_gains.body_vel_vx.kd, 
                  pid_gains.body_vel_vx.tf, 
                  MAIN_LOOP_PERIOD);
                  
    rc_filter_pid(&body_vel_wz_pid, 
                  pid_gains.body_vel_wz.kp, 
                  pid_gains.body_vel_wz.ki, 
                  pid_gains.body_vel_wz.kd, 
                  pid_gains.body_vel_wz.tf, 
                  MAIN_LOOP_PERIOD);
    
    // Enable saturation for all controllers to limit outputs between -1.0 and 1.0
    rc_filter_enable_saturation(&left_wheel_pid, -1.0, 1.0);
    rc_filter_enable_saturation(&right_wheel_pid, -1.0, 1.0);
    rc_filter_enable_saturation(&body_vel_vx_pid, -1.0, 1.0);
    rc_filter_enable_saturation(&body_vel_wz_pid, -1.0, 1.0);
    return MBOT_OK;
}

void mbot_motor_vel_controller(float target_left_vel, float target_right_vel, 
                              float current_left_vel, float current_right_vel,
                              float* left_pwm_out, float* right_pwm_out) {
    float left_error = target_left_vel - current_left_vel;
    float right_error = target_right_vel - current_right_vel;
    
    // Run PID controllers for each wheel
    *left_pwm_out = rc_filter_march(&left_wheel_pid, left_error);
    *right_pwm_out = rc_filter_march(&right_wheel_pid, right_error);
}

void mbot_body_vel_controller(float target_vx, float target_wz,
                             float current_vx, float current_wz,
                             float* vx_pwm, float* wz_pwm) {
    // Calculate errors in body velocity
    float vx_error = target_vx - current_vx;
    float wz_error = target_wz - current_wz;
    
    // Run PID controllers for forward and angular velocity
    *vx_pwm = rc_filter_march(&body_vel_vx_pid, vx_error);
    *wz_pwm = rc_filter_march(&body_vel_wz_pid, wz_error);
}

int init_parameter_server(rclc_parameter_server_t* parameter_server, rcl_node_t* node) {
    rcl_ret_t ret;
    
    // Initialize parameter server with options for low memory mode
    rclc_parameter_options_t options = {
        .notify_changed_over_dds = false,
        .max_params = 16,  
        .allow_undeclared_parameters = false,
        .low_mem_mode = false
    };
    
    ret = rclc_parameter_server_init_with_option(parameter_server, node, &options);
    if (ret != RCL_RET_OK) {
        printf("[FATAL] Failed to init parameter server: %d\n", ret);
        return MBOT_ERROR;
    }

    // Add parameters for all PID gains
    ret = rclc_add_parameter(parameter_server, "left_wheel.kp", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "left_wheel.ki", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "left_wheel.kd", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "left_wheel.tf", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_add_parameter(parameter_server, "right_wheel.kp", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "right_wheel.ki", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "right_wheel.kd", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "right_wheel.tf", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_add_parameter(parameter_server, "body_vel_vx.kp", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "body_vel_vx.ki", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "body_vel_vx.kd", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "body_vel_vx.tf", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_add_parameter(parameter_server, "body_vel_wz.kp", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "body_vel_wz.ki", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "body_vel_wz.kd", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "body_vel_wz.tf", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    // Set initial values for all PID gains from pid_gains
    ret = rclc_parameter_set_double(parameter_server, "left_wheel.kp", pid_gains.left_wheel.kp);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "left_wheel.ki", pid_gains.left_wheel.ki);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "left_wheel.kd", pid_gains.left_wheel.kd);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "left_wheel.tf", pid_gains.left_wheel.tf);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_parameter_set_double(parameter_server, "right_wheel.kp", pid_gains.right_wheel.kp);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "right_wheel.ki", pid_gains.right_wheel.ki);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "right_wheel.kd", pid_gains.right_wheel.kd);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "right_wheel.tf", pid_gains.right_wheel.tf);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_parameter_set_double(parameter_server, "body_vel_vx.kp", pid_gains.body_vel_vx.kp);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "body_vel_vx.ki", pid_gains.body_vel_vx.ki);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "body_vel_vx.kd", pid_gains.body_vel_vx.kd);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "body_vel_vx.tf", pid_gains.body_vel_vx.tf);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_parameter_set_double(parameter_server, "body_vel_wz.kp", pid_gains.body_vel_wz.kp);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "body_vel_wz.ki", pid_gains.body_vel_wz.ki);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "body_vel_wz.kd", pid_gains.body_vel_wz.kd);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "body_vel_wz.tf", pid_gains.body_vel_wz.tf);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    return MBOT_OK;
}

bool parameter_callback(const Parameter * old_param, const Parameter * new_param, void * context) {
    if (new_param == NULL) {
        // Parameter deletion not allowed
        return false;
    }

    const char* param_name = new_param->name.data;

    if (strcmp(param_name, "left_wheel.kp") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.left_wheel.kp = new_param->value.double_value;
            rc_filter_pid(&left_wheel_pid, pid_gains.left_wheel.kp, pid_gains.left_wheel.ki, 
                          pid_gains.left_wheel.kd, pid_gains.left_wheel.tf, MAIN_LOOP_PERIOD);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "left_wheel.ki") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.left_wheel.ki = new_param->value.double_value;
            rc_filter_pid(&left_wheel_pid, pid_gains.left_wheel.kp, pid_gains.left_wheel.ki, 
                          pid_gains.left_wheel.kd, pid_gains.left_wheel.tf, MAIN_LOOP_PERIOD);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "left_wheel.kd") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.left_wheel.kd = new_param->value.double_value;
            rc_filter_pid(&left_wheel_pid, pid_gains.left_wheel.kp, pid_gains.left_wheel.ki, 
                          pid_gains.left_wheel.kd, pid_gains.left_wheel.tf, MAIN_LOOP_PERIOD);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "left_wheel.tf") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.left_wheel.tf = new_param->value.double_value;
            rc_filter_pid(&left_wheel_pid, pid_gains.left_wheel.kp, pid_gains.left_wheel.ki, 
                          pid_gains.left_wheel.kd, pid_gains.left_wheel.tf, MAIN_LOOP_PERIOD);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "right_wheel.kp") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.right_wheel.kp = new_param->value.double_value;
            rc_filter_pid(&right_wheel_pid, pid_gains.right_wheel.kp, pid_gains.right_wheel.ki, 
                          pid_gains.right_wheel.kd, pid_gains.right_wheel.tf, MAIN_LOOP_PERIOD);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "right_wheel.ki") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.right_wheel.ki = new_param->value.double_value;
            rc_filter_pid(&right_wheel_pid, pid_gains.right_wheel.kp, pid_gains.right_wheel.ki, 
                          pid_gains.right_wheel.kd, pid_gains.right_wheel.tf, MAIN_LOOP_PERIOD);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "right_wheel.kd") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.right_wheel.kd = new_param->value.double_value;
            rc_filter_pid(&right_wheel_pid, pid_gains.right_wheel.kp, pid_gains.right_wheel.ki, 
                          pid_gains.right_wheel.kd, pid_gains.right_wheel.tf, MAIN_LOOP_PERIOD);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "right_wheel.tf") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.right_wheel.tf = new_param->value.double_value;
            rc_filter_pid(&right_wheel_pid, pid_gains.right_wheel.kp, pid_gains.right_wheel.ki, 
                          pid_gains.right_wheel.kd, pid_gains.right_wheel.tf, MAIN_LOOP_PERIOD);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "body_vel_vx.kp") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.body_vel_vx.kp = new_param->value.double_value;
            rc_filter_pid(&body_vel_vx_pid, pid_gains.body_vel_vx.kp, pid_gains.body_vel_vx.ki, 
                          pid_gains.body_vel_vx.kd, pid_gains.body_vel_vx.tf, MAIN_LOOP_PERIOD);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "body_vel_vx.ki") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.body_vel_vx.ki = new_param->value.double_value;
            rc_filter_pid(&body_vel_vx_pid, pid_gains.body_vel_vx.kp, pid_gains.body_vel_vx.ki, 
                          pid_gains.body_vel_vx.kd, pid_gains.body_vel_vx.tf, MAIN_LOOP_PERIOD);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "body_vel_vx.kd") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.body_vel_vx.kd = new_param->value.double_value;
            rc_filter_pid(&body_vel_vx_pid, pid_gains.body_vel_vx.kp, pid_gains.body_vel_vx.ki, 
                          pid_gains.body_vel_vx.kd, pid_gains.body_vel_vx.tf, MAIN_LOOP_PERIOD);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "body_vel_vx.tf") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.body_vel_vx.tf = new_param->value.double_value;
            rc_filter_pid(&body_vel_vx_pid, pid_gains.body_vel_vx.kp, pid_gains.body_vel_vx.ki, 
                          pid_gains.body_vel_vx.kd, pid_gains.body_vel_vx.tf, MAIN_LOOP_PERIOD);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "body_vel_wz.kp") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.body_vel_wz.kp = new_param->value.double_value;
            rc_filter_pid(&body_vel_wz_pid, pid_gains.body_vel_wz.kp, pid_gains.body_vel_wz.ki, 
                          pid_gains.body_vel_wz.kd, pid_gains.body_vel_wz.tf, MAIN_LOOP_PERIOD);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "body_vel_wz.ki") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.body_vel_wz.ki = new_param->value.double_value;
            rc_filter_pid(&body_vel_wz_pid, pid_gains.body_vel_wz.kp, pid_gains.body_vel_wz.ki, 
                          pid_gains.body_vel_wz.kd, pid_gains.body_vel_wz.tf, MAIN_LOOP_PERIOD);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "body_vel_wz.kd") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.body_vel_wz.kd = new_param->value.double_value;
            rc_filter_pid(&body_vel_wz_pid, pid_gains.body_vel_wz.kp, pid_gains.body_vel_wz.ki, 
                          pid_gains.body_vel_wz.kd, pid_gains.body_vel_wz.tf, MAIN_LOOP_PERIOD);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "body_vel_wz.tf") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.body_vel_wz.tf = new_param->value.double_value;
            rc_filter_pid(&body_vel_wz_pid, pid_gains.body_vel_wz.kp, pid_gains.body_vel_wz.ki, 
                          pid_gains.body_vel_wz.kd, pid_gains.body_vel_wz.tf, MAIN_LOOP_PERIOD);
            pid_updated = true;
        }
    }

    return pid_updated;
}