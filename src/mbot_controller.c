#include "mbot_controller.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <mbot/defs/mbot_params.h>
#include "config/mbot_classic_config.h"

mbot_pid_config_t pid_gains;

int mbot_controller_init(mbot_pid_config_t* pid_gains) {
    return 0;
}

int mbot_motor_vel_controller(void) {
    return 0;
}

int mbot_body_vel_controller(void) {
    return 0;
}

int init_parameter_server(rclc_parameter_server_t* parameter_server, rcl_node_t* node) {
    rcl_ret_t ret;
    
    // Initialize parameter server with options for low memory mode
    rclc_parameter_options_t options = {
        .notify_changed_over_dds = false,
        .max_params = 12,  
        .allow_undeclared_parameters = false,
        .low_mem_mode = true
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

    ret = rclc_add_parameter(parameter_server, "right_wheel.kp", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "right_wheel.ki", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "right_wheel.kd", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_add_parameter(parameter_server, "body_vel_vx.kp", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "body_vel_vx.ki", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "body_vel_vx.kd", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_add_parameter(parameter_server, "body_vel_wz.kp", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "body_vel_wz.ki", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "body_vel_wz.kd", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    // Set initial values for all PID gains from pid_gains
    ret = rclc_parameter_set_double(parameter_server, "left_wheel.kp", pid_gains.left_wheel.kp);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "left_wheel.ki", pid_gains.left_wheel.ki);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "left_wheel.kd", pid_gains.left_wheel.kd);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_parameter_set_double(parameter_server, "right_wheel.kp", pid_gains.right_wheel.kp);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "right_wheel.ki", pid_gains.right_wheel.ki);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "right_wheel.kd", pid_gains.right_wheel.kd);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_parameter_set_double(parameter_server, "body_vel_vx.kp", pid_gains.body_vel_vx.kp);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "body_vel_vx.ki", pid_gains.body_vel_vx.ki);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "body_vel_vx.kd", pid_gains.body_vel_vx.kd);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_parameter_set_double(parameter_server, "body_vel_wz.kp", pid_gains.body_vel_wz.kp);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "body_vel_wz.ki", pid_gains.body_vel_wz.ki);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "body_vel_wz.kd", pid_gains.body_vel_wz.kd);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    return MBOT_OK;
}

bool parameter_callback(const Parameter * old_param, const Parameter * new_param, void * context) {
    if (new_param == NULL) {
        // Parameter deletion not allowed
        return false;
    }

    const char* param_name = new_param->name.data;
    bool param_updated = false;

    if (strcmp(param_name, "left_wheel.kp") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.left_wheel.kp = new_param->value.double_value;
            param_updated = true;
        }
    } else if (strcmp(param_name, "left_wheel.ki") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.left_wheel.ki = new_param->value.double_value;
            param_updated = true;
        }
    } else if (strcmp(param_name, "left_wheel.kd") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.left_wheel.kd = new_param->value.double_value;
            param_updated = true;
        }
    } else if (strcmp(param_name, "right_wheel.kp") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.right_wheel.kp = new_param->value.double_value;
            param_updated = true;
        }
    } else if (strcmp(param_name, "right_wheel.ki") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.right_wheel.ki = new_param->value.double_value;
            param_updated = true;
        }
    } else if (strcmp(param_name, "right_wheel.kd") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.right_wheel.kd = new_param->value.double_value;
            param_updated = true;
        }
    } else if (strcmp(param_name, "body_vel_vx.kp") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.body_vel_vx.kp = new_param->value.double_value;
            param_updated = true;
        }
    } else if (strcmp(param_name, "body_vel_vx.ki") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.body_vel_vx.ki = new_param->value.double_value;
            param_updated = true;
        }
    } else if (strcmp(param_name, "body_vel_vx.kd") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.body_vel_vx.kd = new_param->value.double_value;
            param_updated = true;
        }
    } else if (strcmp(param_name, "body_vel_wz.kp") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.body_vel_wz.kp = new_param->value.double_value;
            param_updated = true;
        }
    } else if (strcmp(param_name, "body_vel_wz.ki") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.body_vel_wz.ki = new_param->value.double_value;
            param_updated = true;
        }
    } else if (strcmp(param_name, "body_vel_wz.kd") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.body_vel_wz.kd = new_param->value.double_value;
            param_updated = true;
        }
    }
    return param_updated;
}