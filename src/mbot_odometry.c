#include "mbot_odometry.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

void mbot_calculate_odometry(float vx, float vy, float wz, float dt, float* x, float* y, float* theta) {
    // Update pose
    *x += vx * dt * cos(*theta) - vy * dt * sin(*theta);
    *y += vx * dt * sin(*theta) + vy * dt * cos(*theta);
    *theta += wz * dt;

    // Normalize theta to [-pi, pi]
    while (*theta > PI) *theta -= 2.0 * PI;
    while (*theta <= -PI) *theta += 2.0 * PI;
}

void mbot_calculate_gyrodometry(float vx, float vy, float wz, float dt, float gyro_z, float* x, float* y, float* theta) {
    *x += vx * dt * cos(*theta) - vy * dt * sin(*theta);
    *y += vx * dt * sin(*theta) + vy * dt * cos(*theta);

    float delta_odo = wz * dt;
    float delta_gyro = gyro_z * dt;
    float delta_G_O = delta_gyro - delta_odo;
    float thres = 0.02;

    if (fabs(delta_G_O) > thres) {
        // Trust gyro this time
        *theta += delta_gyro;
    } else {
        // Trust odometry more
        *theta += delta_odo;
    }

    // Normalize theta to [-pi, pi]
    while (*theta > PI) *theta -= 2.0 * PI;
    while (*theta <= -PI) *theta += 2.0 * PI;
}