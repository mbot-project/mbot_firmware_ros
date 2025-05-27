#ifndef MBOT_ROS_COMMS_H
#define MBOT_ROS_COMMS_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

// Extern declarations for ROS objects

// Publishers
extern rcl_publisher_t imu_publisher;
extern rcl_publisher_t odom_publisher;
extern rcl_publisher_t mbot_vel_publisher;
extern rcl_publisher_t motor_vel_publisher; 

// Published Messages
extern sensor_msgs__msg__Imu imu_msg;
extern nav_msgs__msg__Odometry odom_msg;
extern geometry_msgs__msg__Twist mbot_vel_msg; 
extern geometry_msgs__msg__Vector3 motor_vel_msg; // x: left (MOT_L), y: right (MOT_R), z: unused (MOT_UNUSED)

// Subscribers
extern rcl_subscription_t cmd_vel_subscriber;
extern rcl_subscription_t motor_vel_cmd_subscriber;
extern rcl_subscription_t motor_pwm_cmd_subscriber;

// Subscription Message Buffers
extern geometry_msgs__msg__Twist cmd_vel_msg_buffer; 
extern geometry_msgs__msg__Vector3 motor_vel_cmd_msg_buffer; // x: left, y: right, z: unused
extern geometry_msgs__msg__Vector3 motor_pwm_cmd_msg_buffer; // x: left, y: right, z: unused

// Initialization functions
int mbot_ros_comms_init_messages(rcl_allocator_t* allocator); 
int mbot_ros_comms_init_publishers(rcl_node_t *node);
int mbot_ros_comms_init_subscribers(rcl_node_t *node);

// Callback function prototypes
void cmd_vel_callback(const void * msgin);
void motor_vel_cmd_callback(const void * msgin);
void motor_pwm_cmd_callback(const void * msgin);

// Helper to add comms to executor
int mbot_ros_comms_add_to_executor(rclc_executor_t *executor);

#endif // MBOT_ROS_COMMS_H 