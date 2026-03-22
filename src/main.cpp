#include <Arduino.h>
#include <Wire.h>
#include "Control_Panel.h"
#include <functional>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define RIDE_CYCLE_TIME 10000 // 10 second ride cycle for testing should be 60000ms later

//State machine
enum class State {
    STOP,
    AWAITING_DISPATCH,
    NORMAL,
    ESTOP,
    POST,
    MAINTENANCE,
};
volatile State current_state = State::ESTOP;

// Timers will be created in setup heartbeat gets started in POST
// This is the ride cycle timer, started when transitioning from AWAITING_DISPATCH to NORMAL, and will be used to track the ride cycle and return to stop state
hw_timer_t *ride_cycle_timer = NULL;
// This is used for sending CAN messages (1KHz) and updating the PID controllers (100 Hz). 
hw_timer_t *heartbeat_timer = NULL;

volatile uint8_t send_data[4] = {0,0,0,0};
volatile uint8_t last_recieved_data[4] = {0,0,0,0};
twai_node_handle_t twai_handle = NULL;

// This will be incremented in the heartbeat timer callback and set to 0 whenever a heartbeat is received from the ESP_H.
volatile uint8_t missed_heartbeats = 0; // If we go into the heartbeat isr and this value is 3 or more we go to ESTOP immediately (connection lost)

static bool IRAM_ATTR twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t* event_data, void* user_ctx){
  twai_frame_t rx_msg = {
    .buffer = last_recieved_data,
    .buffer_len = sizeof(last_recieved_data),
  };
  if(ESP_OK == twai_node_receive_from_isr(handle, &rx_msg)){
    missed_heartbeats = 0;
    Serial.printf("Received CAN message with ID: 0x%X, Data: ", rx_msg.header.id);
    for(int i = 0; i < rx_msg.buffer_len; i++){
      Serial.printf("%d ", rx_msg.buffer[i]);
    }
    Serial.println();
  }
  return true;
}

//TODO: Add Heartbeat ISR for Ian's stuff and sending CAN messages and incrementing missed_heartbeats (also have it trigger E-stop if we miss 3 heartbeats in a row)


void IRAM_ATTR ride_cycle_end(){
    timerStop(ride_cycle_timer);
    if(current_state == State::NORMAL){ // Only transition back to STOP if we're currently in NORMAL state, otherwise we might interrupt an ESTOP or MAINTANENCE cycle
        current_state = State::STOP; // Transition back to STOP state at the end of the ride cycle
    }else{
        current_state = State::ESTOP; // If we're not in NORMAL state at the end of the ride cycle, something went wrong, so transition to ESTOP
    }
}

void IRAM_ATTR estop_isr(){
    current_state = State::ESTOP; // Transition to ESTOP state immediately when the E-stop button is pressed
    // This should also pull enable low on both drivers and pull shutdown low
    // Enable will be part of motor class and shutdown will be in main
}

ControlPanel controlPanel; // Global instance of the control panel still need to call init in setup


bool rideCycleHandler(unsigned long timer_value) {
    return true; //for now
}


// RESET SECTION

// TODO: POST IS GONNA NEED TO BE A TASK THAT SHOULD HAVE PRIORITY OVER LOOP IT WONT RETURN ANYTHING (OR AT ALL)
// IT SHOULD START THE HEARTBEAT PERFROM A BUNCH OF CHECKS USE vTaskDelayUntil TO MAKE SURE ESP_H HAS TIME TO BOOT AND RESPOND
// THEN IT CAN SWITCH STATES TO STOP IF EVERYTHING CHECKS OUT, OR LEAVE US IN ESTOP IF NOT. 
TaskHandle_t POST_task_handle = NULL;

void POST_task(void* pvParameters){
  while(true){

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // The callback for reset should notify this task

    current_state = State::POST;
    // return power
    missed_heartbeats = 0; // Reset missed heartbeats in case we were in ESTOP due to connection issues
    controlPanel.setDisplayText("Running POST...");
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay to allow ESP_H to boot
    // send startup CAN message (startup should be distinct from heartbeats with a lower ID so it has higher prority on the bus)
    // wait for notification from CAN ISR with a reasonable timeout if it timesout, stay in ESTOP
    // otherwise start the heartbeat, and watchdog timers
    // perform any other necessary startup checks here
    // if everything checks out, transition to STOP, otherwise stay in ESTOP
    
    //FOR TESTING
    vTaskDelay(3000 / portTICK_PERIOD_MS); 
    current_state = State::STOP; 
  }
}

void IRAM_ATTR reset_isr(){
    // This is the ISR for the reset button, it will notify the POST task to start the POST process when the reset button is held down for long enough in ESTOP state
    if(current_state == State::ESTOP){
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(POST_task_handle, &xHigherPriorityTaskWoken); // Notify the POST task to start the POST process
        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR(); // If the POST task has a higher priority than the currently running task, have the scheduler switch to it immediately
        }
    }
}

// TODO: Add Ian's class as global vars for the 2 lower motors

void setup() {
    Serial.begin(115200);
    controlPanel.init();
    xTaskCreatePinnedToCore(
        POST_task, // Function to implement the task
        "POST Task", // Name of the task
        4096, // Stack size in words
        nullptr, // Task input parameter
        3, // Priority higher than loop
        &POST_task_handle, // Task handle so we can send notifications to it
        1 // Run on core 1 to keep it separate from the control panel which runs mostly on core 0
    );
    // Set up controll panel interrupts
    controlPanel.setResetCallback(reset_isr); // Set the reset callback to the reset_isr function 
    // TODO: attach up input callback


    //ride_cycle_timer setup
    ride_cycle_timer = timerBegin(1000000); // Create a hardware timer with a prescaler of 80 (1 tick = 1 microsecond)
    timerAttachInterrupt(ride_cycle_timer, ride_cycle_end); // Attach the timer callback
    timerAlarm(ride_cycle_timer, RIDE_CYCLE_TIME * 1000, false, 0); // Set the timer to trigger at the end of every ride cycle and not auto-reload
    timerStop(ride_cycle_timer); // Start with the ride cycle timer stopped, it will be started when transitioning to NORMAL state

    // TODO: Same for heartbeat timer 

}

void loop() {
    // TODO: Implement safety checks that can pull the state into ESTOP (Current monitoring, diag pins, etc.)

    // TODO: Move state_switching logic to the callbacks (post and input are the only ones that should change to a state other than estop)

    //TODO: Add text to the LCD and set LEDs according to the state using the control panel class.
    switch (current_state) {
        case State::STOP:
            Serial.println("Currently in STOP state");
            // if (dispatch_pressed) {
            //     current_state = State::NORMAL;
            //     Serial.println("Transitioning to NORMAL state");
            //     //restart the ride cycle timer
            //     timerRestart(ride_cycle_timer); 
            //     timerAlarm(ride_cycle_timer, RIDE_CYCLE_TIME * 1000, false, 0); // Set the timer to trigger at the end of every ride cycle and not auto-reload
            //     timerStart(ride_cycle_timer); 
            // }
            break;
        case State::NORMAL:
            Serial.println("Currently in NORMAL state");
            // if(!rideCycleHandler(timerRead(ride_cycle_timer))){ // This checks if the ride cycle timer has reached the end of the ride cycle, and if so, it will transition back to STOP state. This is a non-blocking way to handle the ride cycle timing.
            //     current_state = State::STOP;
            //     Serial.println("Ride cycle ended, transitioning back to STOP state");
            // }
            break;
        case State::ESTOP:
            Serial.println("Currently in ESTOP state");
            // TODO: change the analog write to be a function call to the control panel class 

            //analogWrite(ESTOP_LED_PIN, 50); // Turn on the ESTOP LED 
            // if (reset_cycles > RESET_MIN_CYCLES){ // This is the amount of time the button must be held divided by the loop delay time (+10 for the debounce delay) to determine how many cycles the button needs to be held for
            //     reset_cycles = 0; // Reset the cycle count after transitioning to STOP state
            //     if(POST()){
            //         current_state = State::STOP;
            //         Serial.println("Transitioning to STOP state");
            //     }else{
            //         Serial.println("POST failed, remaining in ESTOP state");
            //     }
            // }
            break;
        case State::MAINTENANCE:
            Serial.println("Currently in MAINTENANCE state");
            break;
    }
}

// SemaphoreHandle_t i2cMutex;
// MCP23008 *mcp = nullptr; 

// void updateCallback(uint8_t newState) {
//     Serial.print("MCP23008 Interrupt! New state: ");
//     for (int i = 7; i >= 0; i--) Serial.print((newState >> i) & 1);
//     Serial.println();
//     if(!(newState & 0x02)){ // If the second bit is low, that means the button connected to that pin was pressed (assuming active-low with pull-up)
//         mcp->write_stage(0, true); // Set the first bit high to turn on the LED connected to that pin
//     }else{
//       mcp->write_stage(0, false); // Set the first bit high to turn on the LED connected to that pin
//     }
//     if(!(newState & 0x04)){ // If the third bit is low, that means the button connected to that pin was pressed (assuming active-low with pull-up)
//         mcp->write_stage(3, true); // Set the fourth bit high to turn on the LED connected to that pin
//     }else{
//         mcp->write_stage(3, false); // Set the fourth bit high to turn on the LED connected to that pin
//     }
//     mcp->commit(); // Send the staged changes over I2C
// }

// void setup() {
//     Serial.begin(115200);
//     Wire.begin(SDA, SCL,400000); // Initialize I2C with specified SDA, SCL pins and fast mode (400kHz)
//     i2cMutex = xSemaphoreCreateMutex();
//     mcp = new MCP23008(i2cMutex); // Initialize the MCP23008 instance with the I2C mutex
//     mcp->pinMode_stage(0, PIN_TYPE::PIN_OUTPUT);
//     mcp->pinMode_stage(1, PIN_TYPE::PIN_PULLUP_INTERRUPT); 
//     mcp->pinMode_stage(2, PIN_TYPE::PIN_PULLUP_INTERRUPT); 
//     mcp->pinMode_stage(3, PIN_TYPE::PIN_OUTPUT);
//     mcp->setUpdateCallback(updateCallback); // Set the interrupt callback function
//     mcp->commit();
// }

// void loop(){
//   Serial.println("Looping");
//   vTaskDelay(1000 / portTICK_PERIOD_MS);  
// }

// #include <Arduino.h>
// #include <Wire.h>
// #include "interrupt.h"

// void setup() {
//   init();
// }

// void loop(){
//   if(starting){
//     start();
//   }
//   if(flag){
//     response();
//   }
// }

/*#include <Arduino.h>
#include <Wire.h>

#define EXTADD 0x20
#define IODIR 0x00
#define SCL 22
#define SDA 21
#define OLAT 0x0A
#define GPIO 0x09
void setup() {
  Wire.begin(SDA, SCL);
  Serial.begin(115200);
  Wire.beginTransmission(EXTADD);
  Wire.write(IODIR);
  Wire.write(0x00);
  Wire.endTransmission();
}
 
void loop() {
  Wire.beginTransmission(EXTADD);
  Wire.write(GPIO);
  Wire.write(0xff);
  Wire.endTransmission();
  Serial.println("1");
  delay(1000);
  Wire.beginTransmission(EXTADD);
  Wire.write(GPIO);
  Wire.write(0x00);
  Wire.endTransmission();
  Serial.println("0");
  delay(1000);
}*/