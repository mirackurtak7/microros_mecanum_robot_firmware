#ifndef ESP32_MOTOR_H
#define ESP32_MOTOR_H

#include <Arduino.h>
#include "motor_interface.h"

class ESP32Motor: public MotorInterface
{
    private:
        int in_a_pin_;
        int in_b_pin_;
        int pwm_pin_;
        int pwm_channel_;

    protected:
        void forward(int pwm) override
        {
            digitalWrite(in_a_pin_, HIGH);
            digitalWrite(in_b_pin_, LOW);
            ledcWrite(pwm_channel_, abs(pwm));
        }

        void reverse(int pwm) override
        {
            digitalWrite(in_a_pin_, LOW);
            digitalWrite(in_b_pin_, HIGH);
            ledcWrite(pwm_channel_, abs(pwm));
        }

    public:
        ESP32Motor(float pwm_frequency, int pwm_bits, bool invert, int pwm_pin, int in_a_pin, int in_b_pin, int pwm_channel): 
            MotorInterface(invert),
            in_a_pin_(in_a_pin),
            in_b_pin_(in_b_pin),
            pwm_pin_(pwm_pin),
            pwm_channel_(pwm_channel)
        {
            pinMode(in_a_pin_, OUTPUT);
            pinMode(in_b_pin_, OUTPUT);
            
            // LEDC Setup (replaces analogWrite which might conflict)
            ledcSetup(pwm_channel_, pwm_frequency, pwm_bits);
            ledcAttachPin(pwm_pin_, pwm_channel_);
            
            //ensure that the motor is in neutral state during bootup
            ledcWrite(pwm_channel_, 0);
        }

        void brake() override
        {
            ledcWrite(pwm_channel_, 0);
        }
};

#endif
