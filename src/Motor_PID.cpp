#include "Motor_PID.h"
 
bool MotorPID::_ledc_timer_initialized = false;
 
bool MotorPID::init(int enc_A_pin, int enc_B_pin, int pwm_pin_1, int pwm_pin_2, int enable_pin, float counts_per_rev, ledc_channel_t ledc_channel_1, ledc_channel_t ledc_channel_2, float Kp, float Ki, float Kd, int timestep_ms) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    _prevError = 0;
    _integral = 0;
    _timestep_ms = timestep_ms;
    _enablePin = enable_pin;
    _targetVelocity = 0;
    _targetPosition = 0;
    _countsPerRev = counts_per_rev;
    pinMode(_enablePin, OUTPUT);
    digitalWrite(_enablePin, LOW);
    pinMode(enc_A_pin, INPUT);
    pinMode(enc_B_pin, INPUT);
    pinMode(pwm_pin_1, OUTPUT);
    pinMode(pwm_pin_2, OUTPUT);
    if(!initPCNT(enc_A_pin, enc_B_pin)){
        return false;
    }
    if(!initLEDC(pwm_pin_1, pwm_pin_2, ledc_channel_1, ledc_channel_2)){
        return false;
    }
    return true;
}
 
bool MotorPID::initPCNT(int enc_A_pin, int enc_B_pin) {
    pcnt_unit_config_t unit_cfg = {
        .low_limit   = PCNT_L_LIM,
        .high_limit  = PCNT_H_LIM,
         .flags = {
           .accum_count = true,        // driver accumulates overflows internally
         }
    };
    if(pcnt_new_unit(&unit_cfg, &_pcnt_unit) != ESP_OK){
        return false;
    }
    pcnt_unit_add_watch_point(_pcnt_unit, PCNT_H_LIM);
    pcnt_unit_add_watch_point(_pcnt_unit, PCNT_L_LIM);
    pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = 1000,
    };
    pcnt_unit_set_glitch_filter(_pcnt_unit, &filter_cfg);
 
    pcnt_chan_config_t chan0_cfg = {
        .edge_gpio_num  = enc_A_pin,
        .level_gpio_num = enc_B_pin,
    };
    if(pcnt_new_channel(_pcnt_unit, &chan0_cfg, &_pcnt_chan0) != ESP_OK){
        return false;
    }
 
    pcnt_channel_set_edge_action(_pcnt_chan0,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,   // A rising  → increment
        PCNT_CHANNEL_EDGE_ACTION_DECREASE);  // A falling → decrement
 
    pcnt_channel_set_level_action(_pcnt_chan0,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,      // B high → keep direction
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);  // B low  → flip direction
   
    pcnt_chan_config_t chan1_cfg = {
        .edge_gpio_num  = enc_B_pin,
        .level_gpio_num = enc_A_pin,
    };
    if(pcnt_new_channel(_pcnt_unit, &chan1_cfg, &_pcnt_chan1) != ESP_OK){
        return false;
    }
    pcnt_channel_set_edge_action(_pcnt_chan1,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,   // B rising  → decrement
        PCNT_CHANNEL_EDGE_ACTION_INCREASE);  // B falling → increment
    pcnt_channel_set_level_action(_pcnt_chan1,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_unit_enable(_pcnt_unit);
    pcnt_unit_clear_count(_pcnt_unit);
    pcnt_unit_start(_pcnt_unit);
    return true;
}
 
bool MotorPID::initLEDC(int pwm_pin_1, int pwm_pin_2, ledc_channel_t ledc_channel_1, ledc_channel_t ledc_channel_2) {
    if(!_ledc_timer_initialized) {
        ledc_timer_config_t ledc_timer = {
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_8_BIT, // Set PWM resolution 0-255
            .timer_num = LEDC_TIMER_0,
            .freq_hz = 15000, // 15 kHz should be fine?
        };
        if(ledc_timer_config(&ledc_timer) != ESP_OK){
            return false;
        }
        _ledc_timer_initialized = true;
    }
    ledc_channel_config_t ledc_config_1 = {
        .gpio_num = pwm_pin_1,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = ledc_channel_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0, // Start with motor off
        .hpoint = 0, // Will need to set to something nonzero if using multiple motors/setting high and low
    };
    if(ledc_channel_config(&ledc_config_1) != ESP_OK){
        return false;
    }
    _ledc_channel_1 = ledc_channel_1;
    ledc_channel_config_t ledc_config_2 = {
        .gpio_num = pwm_pin_2,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = ledc_channel_2,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0, // Start with motor off
        .hpoint = 0, // Will need to set to something nonzero if using multiple motors/setting high and low
    };
    if(ledc_channel_config(&ledc_config_2) != ESP_OK){
        return false;
    }
    _ledc_channel_2 = ledc_channel_2;
    return true;
}
 
IRAM_ATTR void MotorPID::update() {
    _last_raw_count = _raw_count;
    pcnt_unit_get_count(_pcnt_unit, &_raw_count);
    if(_velocity_mode){
        //Velocity control        
        float rpm_value = ((_raw_count - _last_raw_count) / _countsPerRev) * (60000.0f / _timestep_ms); // Convert count difference to RPM
       
        // Incremental PI calculations
        _prev_err = _err;
        _err = _targetVelocity - rpm_value;
        _Derr = _err - _prev_err;
        _currentPWM += _Kp * _Derr + _Ki * _err * (_timestep_ms / 1000.0f);
        _currentPWM = _currentPWM < -255.0f ? -255.0f : (_currentPWM > 255.0f ? 255.0f : _currentPWM); // Constrain PWM to valid range
        if(_currentPWM >= 0){
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, _ledc_channel_1, (uint32_t)_currentPWM);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, _ledc_channel_2, 0);
        } else {
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, _ledc_channel_1, 0);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, _ledc_channel_2, (uint32_t)(-_currentPWM));
        }
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, _ledc_channel_1);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, _ledc_channel_2);
    }else{
        // Position control
        float position_value = (_raw_count / _countsPerRev) * 360.0f; // Convert count to degrees
        _prev_err = _err;
        _err = _targetPosition - position_value;
        _Derr = _err - _prev_err;
        _integral += _err * _Ki * (_timestep_ms / 1000.0f);
        _currentPWM += _Kp * _err + _integral + (_Kd * _Derr)/(_timestep_ms / 1000.0f);
        _currentPWM = _currentPWM < -100.0f ? -100.0f : (_currentPWM > 100.0f ? 100.0f : _currentPWM); // Constrain PWM to valid range
        // if (_err < 5.0f && _err > -5.0f) _currentPWM = 0; // If we're within 1.25 degrees of the target, just stop the motor to prevent jitter. You can adjust this threshold as needed.
        // if (_currentPWM > 0.0f && _currentPWM < 25.0f)       _currentPWM = 25.0f;
        // else if (_currentPWM < 0.0f && _currentPWM > -25.0f) _currentPWM = -25.0f;
 
        if(_currentPWM >= 0){
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, _ledc_channel_1, (uint32_t)_currentPWM);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, _ledc_channel_2, 0);
        } else {
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, _ledc_channel_1, 0);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, _ledc_channel_2, (uint32_t)(-_currentPWM));
        }
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, _ledc_channel_1);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, _ledc_channel_2);
    }
}
 
void MotorPID::setGoalVelo(float goal) {
    _targetPosition = 0;
    _targetVelocity = goal;
    _velocity_mode = true;
}
 
void MotorPID::setGoalPos(float goal) {
    _targetVelocity = 0;
    _targetPosition = goal;
    _velocity_mode = false;
}
 
void MotorPID::setTunings(float Kp, float Ki, float Kd) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
}
 
void MotorPID::reset() {
    disable();
    _integral = 0;
    _prevError = 0;
    _err = 0;
    _Derr = 0;
    _currentPWM = 0;
    pcnt_unit_clear_count(_pcnt_unit);
}
 
void MotorPID::enable() {
    digitalWrite(_enablePin, HIGH);
}
 
void IRAM_ATTR MotorPID::disable() {
    digitalWrite(_enablePin, LOW);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, _ledc_channel_1, 0);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, _ledc_channel_2, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, _ledc_channel_1);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, _ledc_channel_2);
}
 
float MotorPID::getPos() {
    float position_value = (_raw_count / _countsPerRev) * 360.0f; // Convert count to degrees
    return position_value;
}
 
float MotorPID::getRPM() {
    float rpm_value = ((_raw_count - _last_raw_count) / _countsPerRev) * (60000.0f / _timestep_ms); // Convert count difference to RPM
    return rpm_value;
}