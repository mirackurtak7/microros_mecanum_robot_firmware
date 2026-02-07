// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CONFIG_H
#define CONFIG_H

// ==================== ROBOT BASE CONFIGURATION ====================
// Mecanum base is a 4-wheel omnidirectional robot
#define LINO_BASE Kinematics::MECANUM

// ==================== MOTOR DRIVER CONFIGURATION ====================
#define USE_BTS7960_MOTOR_DRIVER  // BTS7960 High Power Driver

// ==================== IMU CONFIGURATION ====================
// #define USE_MPU9250_IMU  // Using MPU9250 9-axis IMU
// #define USE_MPU6050_IMU // Using MPU6050 6-axis IMU
#define USE_BNO055_IMU // Using BNO055 9-axis IMU

// ==================== LED PIN ====================
#define LED_PIN 2  // ESP32 built-in LED

// ==================== MOTOR DRIVER PINS ====================
// Motor 1 - Front Left
#define MOTOR1_IN_A 25 // RPWM
#define MOTOR1_IN_B 26 // LPWM
#define MOTOR1_INV false

// Motor 2 - Front Right  
#define MOTOR2_IN_A 14 // RPWM
#define MOTOR2_IN_B 12 // LPWM
#define MOTOR2_INV false

// Motor 3 - Rear Left
#define MOTOR3_IN_A 32 // RPWM
#define MOTOR3_IN_B 33 // LPWM (Cannot use 35 as it is input only)
#define MOTOR3_INV false

// Motor 4 - Rear Right
#define MOTOR4_IN_A 19 // RPWM
#define MOTOR4_IN_B 18 // LPWM 
#define MOTOR4_INV false

// ==================== ENCODER PINS ====================
// Encoder 1 - Front Left
#define MOTOR1_ENCODER_A 34
#define MOTOR1_ENCODER_B 36
#define MOTOR1_ENCODER_INV false

// Encoder 2 - Front Right
#define MOTOR2_ENCODER_A 39
#define MOTOR2_ENCODER_B 15
#define MOTOR2_ENCODER_INV false

// Encoder 3 - Rear Left
#define MOTOR3_ENCODER_A 4
#define MOTOR3_ENCODER_B 16
#define MOTOR3_ENCODER_INV false

// Encoder 4 - Rear Right
#define MOTOR4_ENCODER_A 17
#define MOTOR4_ENCODER_B 21
#define MOTOR4_ENCODER_INV false

// ==================== ENCODER SPECS ====================
// Counts per revolution for each encoder
// Adjust these based on your actual encoder specs
#define COUNTS_PER_REV1 1100  // Motor 1 encoder CPR
#define COUNTS_PER_REV2 1100  // Motor 2 encoder CPR
#define COUNTS_PER_REV3 1100  // Motor 3 encoder CPR
#define COUNTS_PER_REV4 1100  // Motor 4 encoder CPR

// ==================== MOTOR SPECS ====================
#define MOTOR_MAX_RPM 30        // Maximum RPM of the motor
#define MAX_RPM_RATIO 0.85       // Percentage of max RPM to use (0.0 to 1.0)
#define MOTOR_OPERATING_VOLTAGE 24.0  // Motor operating voltage
#define MOTOR_POWER_MAX_VOLTAGE 24.0  // Max voltage from battery/supply

// ==================== ROBOT PHYSICAL SPECS ====================
// Wheel diameter in meters
#define WHEEL_DIAMETER 0.34328   // 343.28mm diameter wheels

// Distance between left and right wheels in meters (wheelbase width)
#define LR_WHEELS_DISTANCE 0.235  // 235mm wheelbase

// Distance between front and back wheels in meters (wheelbase length)
// For mecanum robots, this is used in advanced kinematics
#define FB_WHEELS_DISTANCE 0.60   // 600mm distance between front and back wheels

// ==================== PWM CONFIGURATION ====================
// ESP32 supports flexible PWM configuration
#define PWM_FREQUENCY 20000      // 20kHz PWM frequency
#define PWM_BITS 10              // 10-bit resolution (0-1023)
#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX

// ==================== PID TUNING ====================
// Tune these values based on your robot's performance
#define K_P 0.6     // Proportional gain
#define K_I 0.3     // Integral gain  
#define K_D 0.5     // Derivative gain

#endif
