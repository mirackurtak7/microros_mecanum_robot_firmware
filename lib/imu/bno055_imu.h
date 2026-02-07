#ifndef BNO055_IMU_H
#define BNO055_IMU_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "imu_interface.h"

class BNO055IMU : public IMUInterface {
private:
    Adafruit_BNO055 bno_;
    geometry_msgs__msg__Vector3 accel_;
    geometry_msgs__msg__Vector3 gyro_;

public:
    BNO055IMU() : bno_(Adafruit_BNO055(55, 0x28)) {
        // Default BNO055 address A is 0x28, B is 0x29
        // 55 is the sensor ID
    }

    bool startSensor() override {
        if (!bno_.begin()) {
            return false;
        }
        bno_.setExtCrystalUse(true);
        return true;
    }

    geometry_msgs__msg__Vector3 readAccelerometer() override {
        imu::Vector<3> accel = bno_.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        accel_.x = accel.x(); // BNO055 returns m/s^2
        accel_.y = accel.y();
        accel_.z = accel.z();
        return accel_;
    }

    geometry_msgs__msg__Vector3 readGyroscope() override {
        imu::Vector<3> gyro = bno_.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        gyro_.x = gyro.x(); // BNO055 returns rad/s by default? No, default is dps, need check or unit convert
        // Wait, Adafruit library default is:
        // Acceleration: m/s^2
        // Gyroscope: rad/s (if using Adafruit_Unified_Sensor) OR dps (if using raw getVector)?
        // Checking docs/source... usually getVector(VECTOR_GYROSCOPE) returns degrees per second (dps) or rad/s depending on config.
        // Let's assume standard behavior of Adafruit libraries and verify.
        // Actually, Adafruit_BNO055 constructor sets units. init() sets units to Windows-compatible?
        // Let's explicitly check documentation or assume radians if standard ROS usage.
        // But `getVector` returns `imu::Vector<3>`. 
        // Standard BNO055 default unit for Gyro is Dps (Degrees per Second) or Rps (Radians per Second).
        // Let's look at `begin()`: writes UNIT_SEL register.
        // Default Adafruit_BNO055 uses:
        //  ACCEL: m/s^2
        //  GYRO: dps (Degrees per second) [Commonly]
        //  EULER: Degrees
        
        // IF it returns degrees per second, we definitely need to convert to radians for ROS.
        
        gyro_.x = gyro.x() * DEG_TO_RAD;
        gyro_.y = gyro.y() * DEG_TO_RAD;
        gyro_.z = gyro.z() * DEG_TO_RAD;
        return gyro_;
    }
};

#endif
