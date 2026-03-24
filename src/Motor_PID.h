#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <Arduino.h>
#include "driver/pulse_cnt.h"
#include "driver/ledc.h"


#define PCNT_H_LIM      30000   // Hardware counter high limit before overflow ISR fires
#define PCNT_L_LIM      -30000  // Hardware counter low limit

class MotorPID {
    public:
        MotorPID() = default;
        //This function needs pins for enc_A, enc_B, PWM1, PWM2, and enable, also it needs counts per rev, LEDC channel, and initial PID tunings.
        bool init(int enc_A_pin, int enc_B_pin, int pwm_pin_1, int pwm_pin_2, int enable_pin, float counts_per_rev, ledc_channel_t ledc_channel_1, ledc_channel_t ledc_channel_2, float Kp, float Ki, float Kd, int timestep_ms);
        void update(); 
        void setGoalVelo(float goal); // Sets the target velocity in rpm
        void setGoalPos(float goal); // Sets the target position in degrees
        void setTunings(float Kp, float Ki, float Kd); // Sets the PID tunings
        void reset(); // Resets the PID controller (clears integral and sets previous error to 0, and resets pusle counter)
        void enable(); // Enables the motor (sets enable pin high)
        void disable(); // Disables the motor (sets enable pin low)
        float getPos();
    private:
        volatile float _Kp;
        volatile float _Ki;
        volatile float _Kd;
        int _enablePin;
        int _timestep_ms;
        // One of these should be null (calling setGoalVelo should set _targetPosition to null and vice versa)
        volatile float _targetVelocity;
        volatile float _targetPosition;
        float _countsPerRev;
        bool _velocity_mode = true; 
        volatile float _prevError;
        volatile float _integral;

        volatile float _currentRPM;
        int _last_raw_count = 0;
        int _raw_count = 0;
        volatile float _prev_err = 0.0f; // error term for PID
        volatile float _err = 0.0f; // error term for PID
        volatile float _Derr = 0.0f; // change in error for PID
        volatile float _currentPWM = 0.0f; // Current PWM value being sent to the motor (negative means pwming pin 2)

        pcnt_unit_handle_t _pcnt_unit = NULL;
        pcnt_channel_handle_t _pcnt_chan0 = NULL;
        pcnt_channel_handle_t _pcnt_chan1 = NULL;

        static bool _ledc_timer_initialized;
        ledc_channel_t _ledc_channel_1;
        ledc_channel_t _ledc_channel_2;
        
        bool initPCNT(int enc_A_pin, int enc_B_pin);
        bool initLEDC(int pwm_pin_1, int pwm_pin_2, ledc_channel_t ledc_channel_1, ledc_channel_t ledc_channel_2);
    };




#endif