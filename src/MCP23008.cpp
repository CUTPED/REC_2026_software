#include "MCP23008.h"

bool MCP23008::init(SemaphoreHandle_t i2cMutex) {
    return init(i2cMutex, INT_PIN, EXTADD); // Call the more specific init function with default interrupt pin and address
}

bool MCP23008::init(SemaphoreHandle_t i2cMutex, uint8_t interrupt_pin, uint8_t address) {
    _i2cMutex = i2cMutex;
    _address = address;
    _interrupt_pin = interrupt_pin;

    // Set up interrupt pin
    pinMode(_interrupt_pin, PIN_TYPE::PIN_PULLUP);

    // Create a background task for handling I2C communication
    xTaskCreatePinnedToCore(
        backgroundTaskTrampoline, // Task function
        "MCP23008_I2C_Task", // Name of the task
        2048, // Stack size 
        this, // Parameter to pass to the task
        2, // Priority higher than loop 
        &i2cTaskHandle, // Task handle so we can send notifications to it
        0 // Run on core 0 to keep it separate from loop and other tasks on core 1
    );
    attachInterruptArg(digitalPinToInterrupt(_interrupt_pin), isrTrampoline, this, FALLING); // Falling edge assumed by default
    commit(true); // Force commit to initialize the device
    return true; // For now we assume initialization always succeeds, we could add some checks here to verify communication with the device
}

void MCP23008::setUpdateCallback(void (*callback)(void*, uint8_t), void *context) {
    _updateCallback = nullptr;
    _updateCallbackClass = callback;
    _updateContext = context;
}

void MCP23008::setUpdateCallback(void (*callback)(uint8_t)) {
    _updateCallback = callback;
    _updateCallbackClass = nullptr;
    _updateContext = nullptr;
}

void MCP23008::pinMode_stage(uint8_t pin, PIN_TYPE type) {
    switch (type) {
        case PIN_TYPE::PIN_INPUT:
            _stagedIODIR |= (1 << pin); // Set the bit to 1 for input
            _stagedGPPU &= ~(1 << pin); // Clear the bit to 0 for no pull-up
            _stagedGPINTEN &= ~(1 << pin); // Clear the bit to 0 for no interrupt
            break;
        case PIN_TYPE::PIN_PULLUP:
            _stagedIODIR |= (1 << pin); // Set the bit to 1 for input
            _stagedGPPU |= (1 << pin); // Set the bit to 1 for pull-up
            _stagedGPINTEN &= ~(1 << pin); // Clear the bit to 0 for no interrupt
            break;
        case PIN_TYPE::PIN_INTERRUPT:
            _stagedIODIR |= (1 << pin); // Set the bit to 1 for input
            _stagedGPPU &= ~(1 << pin); // Clear the bit to 0 for no pull-up
            _stagedGPINTEN |= (1 << pin); // Set the bit to 1 for interrupt
            break;
        case PIN_TYPE::PIN_PULLUP_INTERRUPT:
            _stagedIODIR |= (1 << pin); // Set the bit to 1 for input
            _stagedGPPU |= (1 << pin); // Set the bit to 1 for pull-up
            _stagedGPINTEN |= (1 << pin); // Set the bit to 1 for interrupt
            break;
        case PIN_TYPE::PIN_OUTPUT:
            _stagedIODIR &= ~(1 << pin); // Clear the bit to 0 for output
            _stagedGPPU &= ~(1 << pin); // Clear the bit to 0 for no pull-up, not really necessary for output but we'll clear it just in case
            _stagedGPINTEN &= ~(1 << pin); // Clear the bit to 0 for no interrupt, again not necessary for output but we'll clear it just in case
            break;
    }
}

void MCP23008::write_stage(uint8_t pin, bool value) {
    if (value) {
        _state |= (1 << pin); // Set the bit to 1
    } else {
        _state &= ~(1 << pin); // Clear the bit to 0
    }
}

void MCP23008::commit(bool force) {
    xSemaphoreTake(_i2cMutex, portMAX_DELAY);
    if(_stagedIODIR != _iodir || force){
        Wire.beginTransmission(_address);
        Wire.write(IODIR); 
        Wire.write(_stagedIODIR); // Write the staged IODIR value
        Wire.endTransmission();
        _iodir = _stagedIODIR; // Update the current IODIR value after writing
    }
    if(_stagedGPPU != _gppu|| force){
        Wire.beginTransmission(_address);
        Wire.write(GPPU); 
        Wire.write(_stagedGPPU); // Write the staged GPPU value
        Wire.endTransmission();
        _gppu = _stagedGPPU; // Update the current GPPU value after writing
    }
    if(_stagedGPINTEN != _gpinten || force){
        Wire.beginTransmission(_address);
        Wire.write(GPINTEN); 
        Wire.write(_stagedGPINTEN); // Write the staged GPINTEN value
        Wire.endTransmission();
        _gpinten = _stagedGPINTEN; // Update the current GPINTEN value after writing
    }
    if(_state != _stagedState || force){ //This really only matters for outputs, technically the input pins are undefined behavior unless explicitly set
        Wire.beginTransmission(_address);
        Wire.write(GPIO); 
        Wire.write(_state); // Write the current state to the GPIO register
        Wire.endTransmission();
        _state = _stagedState; // Update the current state after writing
    }
    xSemaphoreGive(_i2cMutex);
}

void MCP23008::pinMode(uint8_t pin, PIN_TYPE type) {
    pinMode_stage(pin, type);
    commit();
}

void MCP23008::write(uint8_t pin, bool value) {
    write_stage(pin, value);
    commit();
}

uint8_t MCP23008::read() {
    xSemaphoreTake(_i2cMutex, portMAX_DELAY); 
    Wire.beginTransmission(_address);
    Wire.write(GPIO); // Register to read from
    Wire.endTransmission();
    Wire.requestFrom(_address, (uint8_t)1); // Request 1 byte of data
    uint8_t new_data = Wire.read(); // Read the byte of data
    xSemaphoreGive(_i2cMutex);
    
    _state = new_data;
    return _state;
}

uint8_t MCP23008::getState() {
    return _state;
}


//FreeRTOS and ISR stuff:

void IRAM_ATTR MCP23008::isrTrampoline(void* arg) {
    MCP23008* instance = static_cast<MCP23008*>(arg);
    instance->handleISR();
}

void IRAM_ATTR MCP23008::handleISR() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR(i2cTaskHandle, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR(); // If the background task has a higher priority than the currently running task, have the scheduler switch to it immediately
    }
}

void MCP23008::backgroundTaskTrampoline(void* arg) {
    MCP23008* instance = static_cast<MCP23008*>(arg);
    instance->runBackgroundTask();
}

void MCP23008::runBackgroundTask() {
    while (true) {

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        xSemaphoreTake(_i2cMutex, portMAX_DELAY); 
        Wire.beginTransmission(_address);
        Wire.write(INTCAP); // Register to read from
        Wire.endTransmission();
        Wire.requestFrom(_address, (uint8_t)1); // Request 1 byte of data
        uint8_t new_data = Wire.read(); // Read the byte of data
        xSemaphoreGive(_i2cMutex);
        
        _state = new_data;

        if(_updateContext){
            _updateCallbackClass(_updateContext,_state); // Call the user callback function with the new state and context
        } else if(_updateCallback){
            _updateCallback(_state);
        }
    }
}