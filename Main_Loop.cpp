#include <Arduino.h>


//THESE ARE USED FOR NON-SAFETY CRITICAL CONTROLS
//safety critical stuff like E-stop need to be handled in interrupts
#define NUM_INPUTS 2
#define DISPATCH_BUTTON_PIN 27
#define RESET_BUTTON_PIN 26

//safety critical inputs:
#define ESTOP_BUTTON_PIN 13

#define ESTOP_LED_PIN 12

#define RIDE_CYCLE_TIME 10000 // 10 second ride cycle for testing should be 60000ms later

#define LOOP_DELAY_TIME 10 // This is the delay time for the main loop, which will be used to check button states and update the state machine. 
#define DEBOUNCE_TIME 10 // This is the time in milliseconds that the button needs to be held down to be considered a valid press.
#define RESET_HOLD_TIME 3000 // This is the time in milliseconds that the reset button needs to be held down to transition from ESTOP to stop state. 3000 ms (3 seconds)

const int RESET_MIN_CYCLES = RESET_HOLD_TIME / (LOOP_DELAY_TIME +DEBOUNCE_TIME); // This calculates the number of loop cycles the reset button needs to be held for based on the defined hold time and loop delay time

enum State {
    STOP,
    NORMAL,
    ESTOP,
    MAINTANENCE,
};

volatile State current_state = ESTOP;


//Limit Switch pin

//Here we will put the heartbeat timer


// Array of inputs
int input_pins[NUM_INPUTS] = {DISPATCH_BUTTON_PIN, RESET_BUTTON_PIN};

//Is also where we define an array for io_extended inputs




//This is the ride cycle timer, started when transitioning from STOP to NORMAL, and will be used to track the ride cycle and return to stop state
hw_timer_t *ride_cycle_timer = NULL;

void IRAM_ATTR ride_cycle_end(){
    timerStop(ride_cycle_timer);
    if(current_state == NORMAL){ // Only transition back to STOP if we're currently in NORMAL state, otherwise we might interrupt an ESTOP or MAINTANENCE cycle
        current_state = STOP; // Transition back to STOP state at the end of the ride cycle
    }else{
        current_state = ESTOP; // If we're not in NORMAL state at the end of the ride cycle, something went wrong, so transition to ESTOP
    }
}

void IRAM_ATTR estop_isr(){
    current_state = ESTOP; // Transition to ESTOP state immediately when the E-stop button is pressed
    // Serial.println("E-stop button pressed, transitioning to ESTOP state");
}

bool rideCycleHandler(unsigned long timer_value) {
    return true; //for now
}

bool POST(){
    return true; //for now
}

int reset_cycles = 0; // This variable will track the number of cycles the reset button has been pressed for. This can be used to determine if its been down for 3s


void setup() {
    Serial.begin(115200);
    pinMode(ESTOP_LED_PIN, OUTPUT);
    for (int i = 0; i < NUM_INPUTS; i++) {
        if(input_pins[i] >= 34){ // GPIOs 34-39 are input-only and don't have internal pull-up resistors, so we need to handle them differently
            pinMode(input_pins[i], INPUT); 
        } else {
            pinMode(input_pins[i], INPUT_PULLUP);
        }
    }

    //ride_cycle_timer setup
    ride_cycle_timer = timerBegin(1000000); // Create a hardware timer with a prescaler of 80 (1 tick = 1 microsecond)
    timerAttachInterrupt(ride_cycle_timer, ride_cycle_end); // Attach the timer callback
    timerAlarm(ride_cycle_timer, RIDE_CYCLE_TIME * 1000, false, 0); // Set the timer to trigger at the end of every ride cycle and not auto-reload
    timerStop(ride_cycle_timer); // Start with the ride cycle timer stopped, it will be started when transitioning to NORMAL state

    //E-stop button interrupt setup
    pinMode(ESTOP_BUTTON_PIN, INPUT_PULLUP); // Set the E-stop button pin as input with pull-up resistor
    attachInterrupt(digitalPinToInterrupt(ESTOP_BUTTON_PIN), estop_isr, FALLING); // Attach an interrupt to the E-stop button pin

    // Add interrupt for IO extended inputs:

}

void loop() {
    bool inputs[2] ={false}; // Eventually this will be an array of inputs
    for (int i = 0; i < NUM_INPUTS; i++) {
        inputs[i] = (digitalRead(input_pins[i]) == LOW); // Assuming active-low buttons, this will set the corresponding entry in the inputs array to true if the button is pressed
    }
    delay(DEBOUNCE_TIME); // This delay is for debouncing the buttons, it should be long enough to filter out noise but short enough to keep the system responsive
    for (int i = 0; i < NUM_INPUTS; i++) {
        inputs[i] = inputs[i] && (digitalRead(input_pins[i]) == LOW); // This second read after the delay helps to confirm that the button is still pressed, which can help to reduce false triggers from noise
    }

    //THESE SHOULD BE UPDATED WHENEVER NEW BUTTONS ARE ADDED
    const bool dispatch_pressed = inputs[0]; 
    const bool reset_pressed = inputs[1]; 

    //turn off all LEDs so the state machine can turn them on as needed
    analogWrite(ESTOP_LED_PIN, 0);

    Serial.println("Current Input State:");
    Serial.printf("Dispatch: %d, Reset: %d\n", dispatch_pressed, reset_pressed);
    if(reset_pressed){
        reset_cycles++;
        Serial.printf("Reset button held for %d cycles\n", reset_cycles);
    }else{
        reset_cycles = 0; // Reset the cycle count if the button is released
        Serial.println("Reset button released");
    }

    switch (current_state) {
        case STOP:
            Serial.println("Currently in STOP state");
            if (dispatch_pressed) {
                current_state = NORMAL;
                Serial.println("Transitioning to NORMAL state");
                //restart the ride cycle timer
                timerRestart(ride_cycle_timer); 
                timerAlarm(ride_cycle_timer, RIDE_CYCLE_TIME * 1000, false, 0); // Set the timer to trigger at the end of every ride cycle and not auto-reload
                timerStart(ride_cycle_timer); 
            }
            break;
        case NORMAL:
            Serial.println("Currently in NORMAL state");
            if(!rideCycleHandler(timerRead(ride_cycle_timer))){ // This checks if the ride cycle timer has reached the end of the ride cycle, and if so, it will transition back to STOP state. This is a non-blocking way to handle the ride cycle timing.
                current_state = STOP;
                Serial.println("Ride cycle ended, transitioning back to STOP state");
            }
            break;
        case ESTOP:
            Serial.println("Currently in ESTOP state");
            analogWrite(ESTOP_LED_PIN, 50); // Turn on the ESTOP LED 
            if (reset_cycles > RESET_MIN_CYCLES){ // This is the amount of time the button must be held divided by the loop delay time (+10 for the debounce delay) to determine how many cycles the button needs to be held for
                reset_cycles = 0; // Reset the cycle count after transitioning to STOP state
                if(POST()){
                    current_state = STOP;
                    Serial.println("Transitioning to STOP state");
                }else{
                    Serial.println("POST failed, remaining in ESTOP state");
                }
            }
            break;
        case MAINTANENCE:
            Serial.println("Currently in MAINTANENCE state");
            break;
    }
    delay(LOOP_DELAY_TIME); // This delay is to control the loop timing, it should be short enough to keep the system responsive but long enough to prevent excessive CPU usage. It also helps with debouncing the buttons by providing a consistent time interval for checking button states.
}

