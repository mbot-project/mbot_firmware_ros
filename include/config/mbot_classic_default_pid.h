#ifndef CONFIG_MBOT_CLASSIC_DEFAULT_PID_H
#define CONFIG_MBOT_CLASSIC_DEFAULT_PID_H

#include "../../src/mbot_controller.h"

// Default PID gains (mirrors mbot_classic_pid.yaml)
static const mbot_pid_config_t MBOT_DEFAULT_PID_GAINS = {
    .left_wheel  = { .kp = 0.10f, .ki = 0.00f, .kd = 0.00f, .tf = 0.10f },
    .right_wheel = { .kp = 0.10f, .ki = 0.00f, .kd = 0.00f, .tf = 0.10f },
    .body_vel_vx = { .kp = 0.10f, .ki = 0.00f, .kd = 0.00f, .tf = 0.10f },
    .body_vel_wz = { .kp = 0.10f, .ki = 0.00f, .kd = 0.00f, .tf = 0.10f },
};

static const int MBOT_DEFAULT_CONTROL_MODE = 2; // FF+PID

#endif /* CONFIG_MBOT_CLASSIC_DEFAULT_PID_H */ 