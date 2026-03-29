#include "Control_Panel.h"

bool ControlPanel::init() {
    // I2C mutex
    _i2cMutex = xSemaphoreCreateMutex();
    if (_i2cMutex == NULL) {
        return false;
    }

    // Wire 
    Wire.begin(SDA, SCL); // Fast Mode
    
    // LCD
    _lcd.init();
    _lcd.setPWM(_lcd.REG_ONLY,255);
    //We can set rgb if we want i think idk if the lcd we have can do that??
    //We can't do rgb but we can choose brightness and the blue, gray or green

    // Extenders
    if(!_normal_extender.init(_i2cMutex, N_INT_PIN, N_EXT_ADD)){
        return false;
    }
    if(!_maintanence_extender.init(_i2cMutex, M_INT_PIN, M_EXT_ADD)){
        return false;
    }
    _normal_extender.setUpdateCallback(normalCallbackTrampoline, this);

    _normal_extender.pinMode_stage(N_EXT_PIN_DISP_LED, PIN_TYPE::PIN_OUTPUT);
    _normal_extender.pinMode_stage(N_EXT_PIN_RUNNING_LED, PIN_TYPE::PIN_OUTPUT);
    _normal_extender.pinMode_stage(N_EXT_PIN_STOP_LED, PIN_TYPE::PIN_OUTPUT);
    _normal_extender.pinMode_stage(N_EXT_PIN_E_STOP_LED, PIN_TYPE::PIN_OUTPUT);

    _normal_extender.pinMode_stage(N_EXT_PIN_DISP_BTN, PIN_TYPE::PIN_PULLUP_INTERRUPT);
    _normal_extender.pinMode_stage(N_EXT_PIN_DISP_BTN_2, PIN_TYPE::PIN_INTERRUPT);
    _normal_extender.pinMode_stage(N_EXT_PIN_RESET_BTN, PIN_TYPE::PIN_PULLUP_INTERRUPT);
    _normal_extender.pinMode_stage(N_EXT_PIN_DISP_LOCK, PIN_TYPE::PIN_PULLUP_INTERRUPT);

    _normal_extender.commit();

    _maintanence_extender.setUpdateCallback(maintenanceCallbackTrampoline, this);

    _maintanence_extender.pinMode_stage(M_EXT_UP_BTN, PIN_TYPE::PIN_PULLUP_INTERRUPT);
    _maintanence_extender.pinMode_stage(M_EXT_DOWN_BTN, PIN_TYPE::PIN_PULLUP_INTERRUPT);
    _maintanence_extender.pinMode_stage(M_EXT_LEFT_BTN, PIN_TYPE::PIN_PULLUP_INTERRUPT);
    _maintanence_extender.pinMode_stage(M_EXT_RIGHT_BTN, PIN_TYPE::PIN_PULLUP_INTERRUPT);
    _maintanence_extender.pinMode_stage(M_EXT_OK_BTN, PIN_TYPE::PIN_PULLUP_INTERRUPT);
    _maintanence_extender.pinMode_stage(M_EXT_M_MODE, PIN_TYPE::PIN_PULLUP_INTERRUPT);
    _maintanence_extender.pinMode_stage(M_EXT_OFF_MODE, PIN_TYPE::PIN_PULLUP_INTERRUPT);
    _maintanence_extender.pinMode_stage(M_EXT_NORMAL_MODE, PIN_TYPE::PIN_PULLUP_INTERRUPT);

    _maintanence_extender.commit();
    

    pinMode(LIFT_LIMIT_LOW, INPUT_PULLUP);
    pinMode(LIFT_LIMIT_HIGH, INPUT_PULLUP);
    pinMode(STOP_BTN, INPUT_PULLUP);
    pinMode(POWER_MONITOR_PIN, INPUT_PULLUP);

    attachInterruptArg(digitalPinToInterrupt(LIFT_LIMIT_LOW), inputISRtrampoline, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(LIFT_LIMIT_HIGH), inputISRtrampoline, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(STOP_BTN), inputISRtrampoline, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(POWER_MONITOR_PIN), inputISRtrampoline, this, CHANGE);

    // Reset timer
    _resetTimer = timerBegin(1000000); 
    timerAttachInterruptArg(_resetTimer, resetTimerTrampoline, this); 
    timerAlarm(_resetTimer, _resetHoldTime * 1000, false, 0); 
    timerStop(_resetTimer);

    uint8_t update_1 = _normal_extender.read();
    uint8_t update_2 = _maintanence_extender.read();
    inputISR(); // Read the initial state of the direct inputs`
    normalExtenderCallback(update_1);
    maintenanceExtenderCallback(update_2);
    return true;
}


uint16_t ControlPanel::getState() {
    return _state;
}

void ControlPanel::setDisplayText(const char* line1,bool force) {    
    if(force || (strcmp(line1, _displayLine1) !=0)){
        xSemaphoreTake(_i2cMutex, portMAX_DELAY);
        strcpy(_displayLine1, line1);
        _lcd.clear();
        _lcd.setCursor(0, 0);
        _lcd.print(line1);
        xSemaphoreGive(_i2cMutex);
    }
}

void ControlPanel::setDisplayText(const char* line1, const char* line2,bool force) {    
    if(force || (strcmp(line1, _displayLine1) !=0)){
        xSemaphoreTake(_i2cMutex, portMAX_DELAY);
        strcpy(_displayLine1, line1);
        _lcd.clear();
        _lcd.setCursor(0, 0);
        _lcd.print(line1);
        xSemaphoreGive(_i2cMutex);
    }
    if(force || (strcmp(line2, _displayLine2) !=0)){
        xSemaphoreTake(_i2cMutex, portMAX_DELAY);
        strcpy(_displayLine2, line2);
        _lcd.clear();
        _lcd.setCursor(0, 1);
        _lcd.print(line2);
        xSemaphoreGive(_i2cMutex);
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

void IRAM_ATTR ControlPanel::inputISRtrampoline(void* arg) {
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
    _state = (_state & 0xFFF0) | (new_state & 0x000F); // Update bits 0-3 with the new input state, keep extender states unchanged
    portEXIT_CRITICAL(&_stateMux);
    
    if(_userInputCallback){
        _userInputCallback(_state);
    }
}

void ControlPanel::set_led(uint8_t led_value){
    if(led_value==0){ //if state=stationary
        _normal_extender.write_stage(N_EXT_PIN_DISP_LED, true);
        _normal_extender.write_stage(N_EXT_PIN_STOP_LED, true);
        _normal_extender.write_stage(N_EXT_PIN_RUNNING_LED, false);
        _normal_extender.write_stage(N_EXT_PIN_E_STOP_LED, false);
    }
    else if(led_value==1){ //if state=normal
        _normal_extender.write_stage(N_EXT_PIN_DISP_LED, false);
        _normal_extender.write_stage(N_EXT_PIN_STOP_LED, false);
        _normal_extender.write_stage(N_EXT_PIN_RUNNING_LED, true);
        _normal_extender.write_stage(N_EXT_PIN_E_STOP_LED, false);
    }
    else if(led_value==2){ //if state=ESTOP
        _normal_extender.write_stage(N_EXT_PIN_DISP_LED, false);
        _normal_extender.write_stage(N_EXT_PIN_STOP_LED, false);
        _normal_extender.write_stage(N_EXT_PIN_RUNNING_LED, false);
        _normal_extender.write_stage(N_EXT_PIN_E_STOP_LED, true);
    }
    else if(led_value==3){ //if state=Maintenance
        _normal_extender.write_stage(N_EXT_PIN_DISP_LED, true);
        _normal_extender.write_stage(N_EXT_PIN_STOP_LED, false);
        _normal_extender.write_stage(N_EXT_PIN_RUNNING_LED, false);
        _normal_extender.write_stage(N_EXT_PIN_E_STOP_LED, false);
    }
    _normal_extender.commit();
}
