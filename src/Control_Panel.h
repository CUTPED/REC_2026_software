#ifndef CONTROL_PANEL_H
#define CONTROL_PANEL_H

#include <Arduino.h>
#include <Wire.h>
#include <functional>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "MCP23008.h"
#include "DFRobot_RGBLCD1602.h"

// I2C
#define SCL 22
#define SDA 21
#define N_EXT_ADD 0x20
#define M_EXT_ADD 0x24
#define LCD_ADD 0x7C
#define RGB_ADD 0xC0

// Pin Assignments

#define N_INT_PIN 17 //Normal Extender interrupt pin
#define M_INT_PIN 25 //Maintanence Extender interrupt pin
// Normal Extender Outputs
#define N_EXT_PIN_DISP_LED 0
#define N_EXT_PIN_RUNNING_LED 1
#define N_EXT_PIN_STOP_LED 2
#define N_EXT_PIN_E_STOP_LED 3
// Normal Extender Inputs
#define N_EXT_PIN_DISP_BTN 4
#define N_EXT_PIN_DISP_LOCK 5
#define N_EXT_PIN_RESET_BTN 6
#define N_EXT_PIN_DISP_BTN_2 7

//Maintenance Extender Outputs
#define M_EXT_UP_BTN 0
#define M_EXT_DOWN_BTN 1
#define M_EXT_LEFT_BTN 2
#define M_EXT_RIGHT_BTN 3
#define M_EXT_OK_BTN 4
#define M_EXT_M_MODE 5
#define M_EXT_OFF_MODE 6
#define M_EXT_NORMAL_MODE 7

// Direct Input Pins
#define LIFT_LIMIT_LOW 14
#define LIFT_LIMIT_HIGH 2
#define STOP_BTN 5
#define POWER_MONITOR_PIN 23

//For these variables a 0 represents a button that is pressed or a limit switch that is triggered, and a 1 represents a button that is not pressed or a limit switch that is not triggered
    // bit 15: 0 for normal mode, 1 otherwise
    // bit 14: 0 for maintanence mode, 1 otherwise
    // bit 13: 0 for off mode, 1 otherwise
    // bit 12: maintainence mode confirm button
    // bit 11: right button
    // bit 10: left button
    // bit 9: down button
    // bit 8: up button
    // bit 7: dispatch button panel 2
    // bit 6: reset button
    // bit 5: dispatch lock (key)
    // bit 4: dispatch button panel 1
    // bit 3: Ride Power (1 --> power on, 0 --> power cut)
    // bit 2: Stop button 
    // bit 1: Upper Limit Switch
    // bit 0: Lower Limit Switch

// Defined values for reading the states
#define NORMAL_MODE_STATE_BIT 15
#define MAINTENANCE_MODE_STATE_BIT 14
#define OFF_MODE_STATE_BIT 13
#define MAINTENANCE_CONFIRM_BIT 12
#define RIGHT_BTN_BIT 11
#define LEFT_BTN_BIT 10
#define DOWN_BTN_BIT 9
#define UP_BTN_BIT 8
#define DISPATCH_BTN_2_BIT 7
#define RESET_BTN_BIT 6
#define DISPATCH_LOCK_BIT 5
#define DISPATCH_BTN_1_BIT 4
#define POWER_STATE_BIT 3
#define STOP_BTN_BIT 2
#define UPPER_LIMIT_BIT 1
#define LOWER_LIMIT_BIT 0


class ControlPanel {
    public:
        ControlPanel() = default;
        bool init(); 
        uint16_t getState(); 
        void setDisplayText(const char* line1,bool force = false); // This will set the text to be displayed on the LCD, if force is true it will update the display immediately, otherwise it will compare to the cached display text and only update if it has changed to reduce I2C traffic
        void setDisplayText(const char* line1,const char* line2, bool force = false); // This will set the text to be displayed on the LCD, if force is true it will update the display immediately, otherwise it will compare to the cached display text and only update if it has changed to reduce I2C traffic
        void setInputCallback(void (*callback)(uint16_t)); // This will be called as an ISR when any of the inputs change state and get the whole state (it is not an ISR for extended inputs)
        void setResetCallback(void (*callback)()); // This will be called when the reset button is held down. In principle this should be set to the POST function 
        bool setResetHoldTime(uint16_t time_ms); // This will set the amount of time the reset button needs to be held down to trigger the reset callback
        //TODO: handle LED outputs
        void inputISR(); // This will be called when inputs change state, it will update the state variable and call the user input callback if set 

    private:
        DFRobot_RGBLCD1602 _lcd{RGB_ADD, 16, 2, &Wire, LCD_ADD};
        MCP23008 _normal_extender;
        MCP23008 _maintanence_extender;
        SemaphoreHandle_t _i2cMutex;

        //For these variables a 0 represents a button that is pressed or a limit switch that is triggered, and a 1 represents a button that is not pressed or a limit switch that is not triggered
        uint16_t volatile _state; 
        uint16_t volatile _prevState; // This will store the previous state to detect changes
        char *_displayLine1; // This will store the current text to be displayed on the LCD 
        char *_displayLine2; // This will store the current text to be displayed on the LCD 

        uint16_t _resetHoldTime = 3000; // Default to 3 seconds, this is the amount of time the reset button needs to be held down to trigger the reset callback
        hw_timer_t *_resetTimer;
        bool _resetTimerStarted = false;

        static void normalCallbackTrampoline(void* context, uint8_t state); 
        void normalExtenderCallback(uint8_t state);

        static void maintenanceCallbackTrampoline(void* context, uint8_t state);
        void maintenanceExtenderCallback(uint8_t state);

        static void resetTimerTrampoline(void* arg);
        void resetCallback();
        static void inputISRtrampoline(void* arg);

        //User supplied callbacks
        void (*_userInputCallback)(uint16_t); //Runs for any input change, must be ISR safe, gets the whole state as an argument
        void (*_userResetCallback)(); //Runs when the reset button is held down for the specified amount of time

        portMUX_TYPE _stateMux = portMUX_INITIALIZER_UNLOCKED;
};

#endif