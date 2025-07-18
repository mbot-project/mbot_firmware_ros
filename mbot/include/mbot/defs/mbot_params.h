#ifndef MBOT_PARAM_DEFS_H
#define MBOT_PARAM_DEFS_H

#define MBOT_ERROR -1
#define MBOT_OK 0
#define COMMS_ERROR 0
#define COMMS_OK 1
#define MBOT_TIMEOUT_US 1000000         // 1 second
#define MBOT_ERROR_NO_AGENT        -2  // No CDC1 connection detected

#define SYS_CLOCK       125000 //system clock in kHz
#define PWM_FREQ        10000
#define MAIN_LOOP_HZ            110.0 // Hz of control loop
#define MAIN_LOOP_PERIOD        (1.0f / MAIN_LOOP_HZ)
#define ROS_TIMER_HZ            100.0 // Hz of ROS timer
#define ROS_TIMER_PERIOD        (1.0f / ROS_TIMER_HZ)

#define DIFFERENTIAL_DRIVE 1
#define OMNI_120_DRIVE 2 // 3 omni wheels spaced 120deg
#define ACKERMAN_DRIVE 3

#define PUB_DIV_ENCODERS  2   // 100/2 = 50 Hz
#define PUB_DIV_ODOM      4   // 100/4 = 25 Hz
#define PUB_DIV_BATTERY   4
#define PUB_DIV_TF        4 

typedef struct mbot_params_t{
    int motor_polarity[3];
    int encoder_polarity[3];
    float slope_pos[3];
    float itrcpt_pos[3];
    float slope_neg[3];
    float itrcpt_neg[3];
    float body_vel_vx_pid[4];
    float body_vel_wz_pid[4];
    float left_wheel_vel_pid[4];
    float right_wheel_vel_pid[4];
    int control_mode;
} mbot_params_t;



#endif