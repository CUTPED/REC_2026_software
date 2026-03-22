#include "Control_Panel.h"

bool ControlPanel::init() {
    // I2C mutex
    _i2cMutex = xSemaphoreCreateMutex();
    if (_i2cMutex == NULL) {
        return false;
    }

    // Wire 
    Wire.begin(SDA, SCL,400000); // Fast Mode
    
    // LCD
    _lcd.init();
    //We can set rgb if we want i think idk if the lcd we have can do that??

    // Extenders
    if(!_normal_extender.init(_i2cMutex, N_INT_PIN, N_EXT_ADD)){
        return false;
    }
    if(!_maintanence_extender.init(_i2cMutex, M_INT_PIN, M_EXT_ADD)){
        return false;
    }
    _normal_extender.setUpdateCallback(normalCallbackTrampoline, this);
    _maintanence_extender.setUpdateCallback(maintenanceCallbackTrampoline, this);
    //TODO: explicitly call pinMode to setup all pins on extenders
    //note that using pinMode_stage and commit will be an order of magnitude faster than using pinMode here

    //TODO: pinMode and interrupts for direct inputs (limits, stop button, power monitor)

    // Reset timer
    _resetTimer = timerBegin(1000000); 
    timerAttachInterruptArg(_resetTimer, resetTimerTrampoline, this); 
    timerAlarm(_resetTimer, _resetHoldTime * 1000, false, 0); 
    timerStop(_resetTimer);
    return true;
}


uint16_t ControlPanel::getState() {
    return _state;
}

void ControlPanel::setDisplayText(const char* line1,bool force) {    
    if(force || (strcmp(line1, _displayLine1) !=0)){
        xSemaphoreTake(_i2cMutex, portMAX_DELAY);
        _lcd.clear();
        _lcd.setCursor(0, 0);
        _lcd.print(line1);
        xSemaphoreGive(_i2cMutex);
        strcpy(_displayLine1, line1);
    }
}

void ControlPanel::setDisplayText(const char* line1, const char* line2,bool force) {    
    if(force || (strcmp(line1, _displayLine1) !=0)){
        xSemaphoreTake(_i2cMutex, portMAX_DELAY);
        _lcd.clear();
        _lcd.setCursor(0, 0);
        _lcd.print(line1);
        xSemaphoreGive(_i2cMutex);
        strcpy(_displayLine1, line1);
    }
    if(force || (strcmp(line2, _displayLine2) !=0)){
        xSemaphoreTake(_i2cMutex, portMAX_DELAY);
        _lcd.clear();
        _lcd.setCursor(0, 1);
        _lcd.print(line2);
        xSemaphoreGive(_i2cMutex);
        strcpy(_displayLine2, line2);
    }
}

void ControlPanel::setInputCallback(void (*callback)(uint16_t)) {
    _userInputCallback = callback;
}

void ControlPanel::setResetCallback(void (*callback)()) {
    _userResetCallback = callback;
}

bool ControlPanel::setResetHoldTime(uint16_t time_ms) {
    if(_resetTimerStarted){ 
        return false; 
    }
    _resetHoldTime = time_ms;
    return true;
}

void ControlPanel::normalCallbackTrampoline(void* context, uint8_t state) {
    ControlPanel* panel = static_cast<ControlPanel*>(context);
    panel->normalExtenderCallback(state);
}

void ControlPanel::normalExtenderCallback(uint8_t state) {
    // This function will be called when an interrupt occurs on the normal extender, it will update the state variable and call the user input callback if set
    portENTER_CRITICAL(&_stateMux);
    _prevState = _state;
    _state = (_state & 0xFF0F) | (state & 0xF0); // Update bits 4-8 with the new extender state
    portEXIT_CRITICAL(&_stateMux);

    //reset logic (this both starts and cancels the timer for the reset button)
    if (_state & (1<<RESET_BTN_BIT)){ // 1 --> button released or never pressed
        if(_resetTimerStarted){
            timerStop(_resetTimer); 
            _resetTimerStarted = false;
        }
    } else { // 0 --> button down 
        if(!_resetTimerStarted){
            timerRestart(_resetTimer);
            timerAlarm(_resetTimer, _resetHoldTime * 1000, false,0); 
            timerStart(_resetTimer);
            _resetTimerStarted = true;
        }
    }

    if(_userInputCallback){
        _userInputCallback(_state);
    }
}

void ControlPanel::maintenanceCallbackTrampoline(void* context, uint8_t state) {
    ControlPanel* panel = static_cast<ControlPanel*>(context);
    panel->maintenanceExtenderCallback(state);
}

void ControlPanel::maintenanceExtenderCallback(uint8_t state) {
    // This function will be called when an interrupt occurs on the maintanence extender, it will update the state variable and call the user input callback if set
    uint16_t s = static_cast<uint16_t>(state);
    portENTER_CRITICAL(&_stateMux);
    _prevState = _state;
    _state = (_state & 0x00FF) | (s << 8); // Update bits 9-15 with the new extender state
    portEXIT_CRITICAL(&_stateMux);
    if(_userInputCallback){
        _userInputCallback(_state);
    }
}

void ControlPanel::resetTimerTrampoline(void* arg) {
    ControlPanel* panel = static_cast<ControlPanel*>(arg);
    panel->resetCallback();
}

void ControlPanel::resetCallback() {
    // This function will be called when the reset button is held down for the specified amount of time, it will call the user reset callback if set
    if(_userResetCallback){
        _userResetCallback();
    }
}

void IRAM_ATTR ISRtrampoline(void* arg) {
    ControlPanel* panel = static_cast<ControlPanel*>(arg);
    panel->inputISR();
}

void IRAM_ATTR ControlPanel::inputISR() {
    // This function will be called when any of the inputs change state, it will update the state variable and call the user input callback if set 
    uint16_t new_state = 0;
    new_state |= digitalRead(LIFT_LIMIT_LOW) << 0; // Bit 0: Low limit
    new_state |= digitalRead(LIFT_LIMIT_HIGH) << 1; // Bit 1: High limit
    new_state |= digitalRead(STOP_BTN) << 2; // Bit 2: Stop button
    new_state |= digitalRead(POWER_MONITOR_PIN) << 3; // Bit 3: Power state (1 for power, 0 for no power)

    portENTER_CRITICAL(&_stateMux);
    _prevState = _state;
    _state = (_state & 0xFFF0) | (new_state & 0x00F); // Update bits 0-3 with the new input state, keep extender states unchanged
    portEXIT_CRITICAL(&_stateMux);

    if(_userInputCallback){
        _userInputCallback(_state);
    }
}


