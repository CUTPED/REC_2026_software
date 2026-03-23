#ifndef MCP23008_H
#define MCP23008_H

#include <Arduino.h>
#include <Wire.h>
#include <functional>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// These should be done in the main function
// #define SCL 22
// #define SDA 21

//These are default values and can be overridden in the constructor
#define EXTADD 0x20 
#define INT_PIN 19 

#define IODIR 0x00 //pin direction 0 = output, 1 = input
#define IPOL 0x01 // polarity inversion 0 = normal (1 = High, 0 = Low), 1 = inverted
#define GPINTEN  0x02 // interrupt enable
// 0x03 and 0x04 allow for a different interrupt mode but they are not used here
// 0x05 is a configuration register but again here the default is fine
#define GPPU 0x06
// 0x07 is an interrupt flag register also potentially useful but not for our current needs
#define INTCAP 0x08
#define GPIO 0x09
#define OLAT 0x0A

enum class PIN_TYPE {
    PIN_INPUT,
    PIN_PULLUP,
    PIN_INTERRUPT,
    PIN_PULLUP_INTERRUPT,
    PIN_OUTPUT
};

class MCP23008 {
    public:
        MCP23008() = default;
        bool init(SemaphoreHandle_t i2cMutex);
        bool init(SemaphoreHandle_t i2cMutex,uint8_t interrupt_pin, uint8_t address);
        void setUpdateCallback(void (*callback)(void*, uint8_t), void *context); // This allows the user to set a callback function that will be called whenever an interrupt occurs
        void setUpdateCallback(void (*callback)(uint8_t));
        void pinMode_stage(uint8_t pin, PIN_TYPE type); // This allows us to stage a bunch of changes to reduce i2c transactions
        void write_stage(uint8_t pin, bool value); // This allows us to stage a bunch of changes to reduce i2c transactions
        void commit(bool force = false); // This will send the staged changes over i2c
        void pinMode(uint8_t pin, PIN_TYPE type);
        void write(uint8_t pin, bool value);
        uint8_t read();
        uint8_t getState();
        

    private:
        SemaphoreHandle_t _i2cMutex;
        uint8_t _address;
        uint8_t _interrupt_pin;


        uint8_t _stagedState = 0x00;
        uint8_t _stagedIODIR = 0xFF; // Default to all inputs
        uint8_t _stagedGPPU = 0x00; // Default to no pull-ups
        uint8_t _stagedGPINTEN = 0x00; // Default to no interrupts
        
        uint8_t _state;
        uint8_t _iodir; 
        uint8_t _gppu;
        uint8_t _gpinten;
        
        //These let the ISR have the this pointer
        static void isrTrampoline(void* arg); 
        static void backgroundTaskTrampoline(void* arg);

        //These are actually part of the class
        void handleISR();
        void runBackgroundTask();

        //user supplied function for when a button is pressed, gets the current state of the inputs as an argument 
        void (*_updateCallback)(uint8_t) = nullptr;
        void (*_updateCallbackClass)(void*, uint8_t) = nullptr;
        void *_updateContext = nullptr; // This can be used to store any context the user wants to pass to the callback function, it will be passed as an argument to the callback function when it is called
        //Task for reading and calling user function
        TaskHandle_t i2cTaskHandle = NULL;
};



#endif