#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32.h>

// Robot components
#include "config.h"
#include "pid.h"
#include "kinematics.h"
#include "odometry.h"
#include "imu.h"
#include "encoder.h"

// ================== MOTOR PINS & CHANNELS ==================
#define MOTOR1_PWM_A_CHANNEL 0
#define MOTOR1_PWM_B_CHANNEL 1
#define MOTOR2_PWM_A_CHANNEL 2
#define MOTOR2_PWM_B_CHANNEL 3
#define MOTOR3_PWM_A_CHANNEL 4
#define MOTOR3_PWM_B_CHANNEL 5
#define MOTOR4_PWM_A_CHANNEL 6
#define MOTOR4_PWM_B_CHANNEL 7

// ================== GLOBAL OBJECTS ==================

// Encoders (Pointers for Lazy Init)
Encoder* motor1_encoder = nullptr;
Encoder* motor2_encoder = nullptr;
Encoder* motor3_encoder = nullptr;
Encoder* motor4_encoder = nullptr;

// PID Controllers
PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

// Kinematics
Kinematics kinematics(
    Kinematics::LINO_BASE, 
    MOTOR_MAX_RPM, 
    MAX_RPM_RATIO, 
    MOTOR_OPERATING_VOLTAGE, 
    MOTOR_POWER_MAX_VOLTAGE, 
    WHEEL_DIAMETER, 
    LR_WHEELS_DISTANCE + FB_WHEELS_DISTANCE
);

// Target RPMs (Set by cmd_vel)
Kinematics::rpm req_rpm;

// Odometry & IMU
Odometry odometry;
IMU lino_imu;

// micro-ROS Objects
rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_subscription_t twist_subscriber;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 2
#define CONTROL_LOOP_PERIOD_MS 20 // 50 Hz

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// ================== HELPER FUNCTIONS ==================

void moveMotor(int motor_id, int pwm, bool invert)
{
    int in_a_channel, in_b_channel;
 
    // Assign channels based on ID
    if(motor_id == 1) { in_a_channel = MOTOR1_PWM_A_CHANNEL; in_b_channel = MOTOR1_PWM_B_CHANNEL; }
    else if(motor_id == 2) { in_a_channel = MOTOR2_PWM_A_CHANNEL; in_b_channel = MOTOR2_PWM_B_CHANNEL; }
    else if(motor_id == 3) { in_a_channel = MOTOR3_PWM_A_CHANNEL; in_b_channel = MOTOR3_PWM_B_CHANNEL; }
    else if(motor_id == 4) { in_a_channel = MOTOR4_PWM_A_CHANNEL; in_b_channel = MOTOR4_PWM_B_CHANNEL; }
    else return;
 
    if(invert) pwm *= -1;
 
    if(pwm > 0)
    {
        ledcWrite(in_a_channel, abs(pwm));
        ledcWrite(in_b_channel, 0);
    }
    else if(pwm < 0)
    {
        ledcWrite(in_a_channel, 0);
        ledcWrite(in_b_channel, abs(pwm));
    }
    else
    {
        ledcWrite(in_a_channel, 0);
        ledcWrite(in_b_channel, 0);
    }
}

// ================== ROS CALLBACKS ==================

void subscription_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    
    // Calculate required RPM for each motor based on Twist message
    req_rpm = kinematics.getRPM(
        msg->linear.x, 
        msg->linear.y, 
        msg->angular.z
    );
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
        // 1. Get current RPMs from Encoders
        float current_rpm1 = motor1_encoder->getRPM();
        float current_rpm2 = motor2_encoder->getRPM();
        float current_rpm3 = motor3_encoder->getRPM();
        float current_rpm4 = motor4_encoder->getRPM();

        // 2. Compute PID output
        int pwm1 = motor1_pid.compute(req_rpm.motor1, current_rpm1);
        int pwm2 = motor2_pid.compute(req_rpm.motor2, current_rpm2);
        int pwm3 = motor3_pid.compute(req_rpm.motor3, current_rpm3);
        int pwm4 = motor4_pid.compute(req_rpm.motor4, current_rpm4);

        // 3. Drive Motors
        moveMotor(1, pwm1, MOTOR1_INV);
        moveMotor(2, pwm2, MOTOR2_INV);
        moveMotor(3, pwm3, MOTOR3_INV);
        moveMotor(4, pwm4, MOTOR4_INV);
        
        // 4. Odometry update based on real velocities
        Kinematics::velocities current_vel = kinematics.getVelocities(
            current_rpm1, current_rpm2, current_rpm3, current_rpm4
        );
        
        // Update Odom Data (dt in seconds)
        odometry.update(
            CONTROL_LOOP_PERIOD_MS / 1000.0, 
            current_vel.linear_x, 
            current_vel.linear_y, 
            current_vel.angular_z
        );

        // 5. Publish Odometry
        odom_msg = odometry.getData();
        struct timespec time_stamp = {0};
        clock_gettime(CLOCK_REALTIME, &time_stamp);
        odom_msg.header.stamp.sec = time_stamp.tv_sec;
        odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
        RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));

        // 6. Publish IMU
        imu_msg = lino_imu.getData();
        imu_msg.header.stamp.sec = time_stamp.tv_sec;
        imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;
        RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    }
}

// ================== SETUP ==================

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // Booting...
    
    Serial.begin(115200);

    // 1. Initialize Encoders (Lazy Init)
    motor1_encoder = new Encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
    motor2_encoder = new Encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
    motor3_encoder = new Encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
    motor4_encoder = new Encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

    // 2. Initialize Motors (Manual LEDC)
    // Motor 1
    ledcSetup(MOTOR1_PWM_A_CHANNEL, PWM_FREQUENCY, PWM_BITS);
    ledcSetup(MOTOR1_PWM_B_CHANNEL, PWM_FREQUENCY, PWM_BITS);
    ledcAttachPin(MOTOR1_IN_A, MOTOR1_PWM_A_CHANNEL);
    ledcAttachPin(MOTOR1_IN_B, MOTOR1_PWM_B_CHANNEL);

    // Motor 2
    ledcSetup(MOTOR2_PWM_A_CHANNEL, PWM_FREQUENCY, PWM_BITS);
    ledcSetup(MOTOR2_PWM_B_CHANNEL, PWM_FREQUENCY, PWM_BITS);
    ledcAttachPin(MOTOR2_IN_A, MOTOR2_PWM_A_CHANNEL);
    ledcAttachPin(MOTOR2_IN_B, MOTOR2_PWM_B_CHANNEL);

    // Motor 3
    ledcSetup(MOTOR3_PWM_A_CHANNEL, PWM_FREQUENCY, PWM_BITS);
    ledcSetup(MOTOR3_PWM_B_CHANNEL, PWM_FREQUENCY, PWM_BITS);
    ledcAttachPin(MOTOR3_IN_A, MOTOR3_PWM_A_CHANNEL);
    ledcAttachPin(MOTOR3_IN_B, MOTOR3_PWM_B_CHANNEL);

    // Motor 4
    ledcSetup(MOTOR4_PWM_A_CHANNEL, PWM_FREQUENCY, PWM_BITS);
    ledcSetup(MOTOR4_PWM_B_CHANNEL, PWM_FREQUENCY, PWM_BITS);
    ledcAttachPin(MOTOR4_IN_A, MOTOR4_PWM_A_CHANNEL);
    ledcAttachPin(MOTOR4_IN_B, MOTOR4_PWM_B_CHANNEL);

    // 3. Initialize IMU
    bool imu_ret = lino_imu.init();
    if (!imu_ret) {
        // error_loop(); // Proceed even if IMU fails to allow micro-ROS connection
    }

    // 4. micro-ROS Setup
    set_microros_serial_transports(Serial);
    delay(2000);

    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "linorobot_esp32_node", "", &support));

    // Publishers
    RCCHECK(rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered"));

    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"));

    // Subscriber
    RCCHECK(rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    // Timer (20Hz)
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(CONTROL_LOOP_PERIOD_MS), 
        timer_callback));

    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &subscription_callback, ON_NEW_DATA));

    digitalWrite(LED_PIN, LOW); // Ready
}

void loop() {
    delay(10);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}