#include <Arduino.h>
#include <Wire.h>
#include "Control_Panel.h"
#include "Motor_PID.h"
#include <functional>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "driver/pulse_cnt.h"
#include "driver/ledc.h"

#define RIDE_CYCLE_TIME 60000 // Total time for the ride cycle in milliseconds (1 minute)
#define SPIN_UP_TIME 20000 // Time for spin up phase
#define SPIN_DOWN_TIME 20000 // Time for spin down phase

#define MAX_LIFT_POS 3600.0f // Maximum lift position in degrees
#define MAX_CENTRAL_AXIS_RPM 54.0f // Maximum central axis RPM
#define MAX_SECONDARY_AXIS_RPM 135.0f // Maximum secondary axis RPM 

#define CENTER_MOTOR_ENCODER_A 36
#define CENTER_MOTOR_ENCODER_B 39
#define LIFT_MOTOR_ENCODER_A 34
#define LIFT_MOTOR_ENCODER_B 35

#define CENTER_MOTOR_PWM_1 19
#define CENTER_MOTOR_PWM_2 18

#define LIFT_MOTOR_PWM_1 32
#define LIFT_MOTOR_PWM_2 33

#define ENABLE_PIN 12

#define LIFT_MOTOR_CPR 7974.4
#define CENTRAL_AXIS_CPR 7974.4

#define LIFT_MOTOR_KP 0.5f
#define LIFT_MOTOR_KI 0.1f
#define LIFT_MOTOR_KD 0.05f

#define CENTRAL_AXIS_KP 0.5f
#define CENTRAL_AXIS_KI 0.1f
#define CENTRAL_AXIS_KD 0.05f


//State machine
enum class State: uint8_t {
    ESTOP = 5,
    POST = 1,
    STATIONARY = 2,
    NORMAL = 3,
    STOPPING = 4,
    MAINTENANCE = 6,
};
volatile State current_state = State::ESTOP;

// Timers will be created in setup heartbeat gets started in POST
// This is the ride cycle timer, started when transitioning from AWAITING_DISPATCH to NORMAL, and will be used to track the ride cycle and return to stop state
hw_timer_t *ride_cycle_timer = NULL;
// This is used for sending CAN messages (1KHz) and updating the PID controllers (100 Hz). 
hw_timer_t *heartbeat_timer = NULL;

MotorPID LiftMotor;
MotorPID Central_Axis_Motor;

// This will be incremented in the heartbeat timer callback and set to 0 whenever a heartbeat is received from the ESP_H.
fuint8_t missed_heartbeats = 0; // If we go into the heartbeat isr and this value is 3 or more we go to ESTOP immediately (connection lost)

void IRAM_ATTR estop_isr(){
    current_state = State::ESTOP; // Transition to ESTOP state immediately when the E-stop button is pressed
    // TODO: This should also pull enable low on both drivers and pull shutdown low
    // Enable will be part of motor class and shutdown will be in main
}

volatile float secondary_motor_rpm_value = 0.0f; 

static bool IRAM_ATTR twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t* event_data, void* user_ctx){
    uint8_t rx_data[4];
  twai_frame_t rx_msg = {
    .buffer = rx_data,
    .buffer_len = sizeof(rx_data),
  };
  if(ESP_OK == twai_node_receive_from_isr(handle, &rx_msg)){
    //TODO: Compare recieved state to current state and stop if they dont match dont reset the missed heartbeat counter
    if(rx_msg.header.id == 0x100){ // This is a heartbeat message
        if(static_cast<uint8_t>(current_state) == rx_msg.buffer[0]){ // If the state sent by the ESP_H doesn't match our current state, something is wrong, so go to ESTOP
            missed_heartbeats = 0;
        }
    }else{
        // TODO: others? There might be startup stuff this should also be for error frames
    }
    // Serial.printf("Received CAN message with ID: 0x%X, Data: ", rx_msg.header.id);
    for(int i = 0; i < rx_msg.buffer_len; i++){
      Serial.printf("%d ", rx_msg.buffer[i]);
    }
    Serial.println();
  }
  return true;
}

uint8_t heartbeat_data[4] = {0,0,0,0}; 
twai_node_handle_t twai_handle = NULL;
twai_frame_t heartbeat_msg = {
    .header ={
      .id = 0x100, // CAN message ID lower values are higher priority on the bus
      .ide = false, // This just means don't use extended frame format 
    },
    .buffer = (uint8_t*)heartbeat_data, // Point to the data we want to send
    .buffer_len = sizeof(heartbeat_data), // This just specifies the length of the data we're sending, which is 1 byte in this case
};

uint8_t pid_update_counter = 0;

void IRAM_ATTR heartbeat_timer_callback(){
    if (pid_update_counter >= 10){ // Update PID controllers every 10ms (100 Hz)
        LiftMotor.update();
        Central_Axis_Motor.update();
        pid_update_counter = 0;
    }

    missed_heartbeats++;
    if(missed_heartbeats >= 3){
        //ERROR_CODE = n // TODO: set an error code for missed heartbeats that can be sent over CAN and displayed on the LCD
        estop_isr(); // If we miss 3 heartbeats in a row, go to ESTOP
    }
    heartbeat_data[0] = static_cast<uint8_t>(current_state); // Send the current state in the heartbeat message, you can also add other data here if needed
    heartbeat_data[1] = (uint8_t)secondary_motor_rpm_value; 
    heartbeat_data[2] = 0; // These bits will be used in maintainence mode
    heartbeat_data[3] = 0; 
    memcpy(heartbeat_msg.buffer, (uint8_t*)heartbeat_data, sizeof(heartbeat_data)); // Update the data field of the CAN message with the current value of x
    ESP_ERROR_CHECK(twai_node_transmit(twai_handle, &heartbeat_msg,0));

    pid_update_counter++;
}

//TODO: Maintainence mode CAN messages are prolly important

//FIXME: This sucks.
void IRAM_ATTR ride_cycle_end(){
    timerStop(ride_cycle_timer);
    if(current_state == State::NORMAL){ // Only transition back to STOP if we're currently in NORMAL state, otherwise we might interrupt an ESTOP or MAINTANENCE cycle
        current_state = State::STATIONARY; // Transition back to STOP state at the end of the ride cycle
    }else{
        current_state = State::ESTOP; // If we're not in NORMAL state at the end of the ride cycle, something went wrong, so transition to ESTOP
    }
}

ControlPanel controlPanel; // Global instance of the control panel still need to call init in setup

//Ride cycle stuff will be called in loop and will update target postions and velocities for motors
void spinUpCycle(unsigned long timer_value){
    // This function will be called during the spin up phase of the ride cycle, it should ramp up the motors to their target speeds/positions over the course of the spin up time
    // For example, you could use a simple linear ramp like this:
    float ramp_percentage = (float)timer_value / SPIN_UP_TIME;
    LiftMotor.setGoalPos(ramp_percentage * MAX_LIFT_POS);
    Central_Axis_Motor.setGoalVelo(ramp_percentage * MAX_CENTRAL_AXIS_RPM);
    secondary_motor_rpm_value = ramp_percentage * MAX_SECONDARY_AXIS_RPM;
}

void normalRideCycle(unsigned long timer_value){
    // This function will be called during the normal phase of the ride cycle, it should set the motors to their target speeds/positions for the normal ride cycle
    LiftMotor.setGoalPos(MAX_LIFT_POS);
    Central_Axis_Motor.setGoalVelo(MAX_CENTRAL_AXIS_RPM);
    secondary_motor_rpm_value = MAX_SECONDARY_AXIS_RPM;
}

void spinDownCycle(unsigned long timer_value){
    // This function will be called during the spin down phase of the ride cycle, it should ramp down the motors to 0 over the course of the spin down time
    float ramp_percentage = 1.0f - ((float)timer_value / SPIN_DOWN_TIME);
    LiftMotor.setGoalPos(ramp_percentage * MAX_LIFT_POS);
    Central_Axis_Motor.setGoalVelo(ramp_percentage * MAX_CENTRAL_AXIS_RPM);
    secondary_motor_rpm_value = ramp_percentage * MAX_SECONDARY_AXIS_RPM;
}

bool rideCycleHandler(unsigned long timer_value) {
    if(timer_value <= SPIN_UP_TIME){
        spinUpCycle(timer_value);
    }else if(timer_value <= RIDE_CYCLE_TIME - SPIN_DOWN_TIME){
        normalRideCycle(timer_value - SPIN_UP_TIME);
    }else if(timer_value <= RIDE_CYCLE_TIME){
        spinDownCycle(timer_value - (RIDE_CYCLE_TIME - SPIN_DOWN_TIME));
    } else{
        return false; // Ride cycle is over
    }
    return true; 
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
    //TODO: after supplying power, waiting and send the startup CAN message, we can wait for a notificaition from the recieve callback.
    //FOR TESTING
    vTaskDelay(3000 / portTICK_PERIOD_MS); 
    current_state = State::STATIONARY; 
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

//TODO: Maintainence Mode functions (This might become part of the input callback with a big if at the top)

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

    //Motor Initialization
    LiftMotor.init(LIFT_MOTOR_ENCODER_A, LIFT_MOTOR_ENCODER_B, LIFT_MOTOR_PWM_1, LIFT_MOTOR_PWM_2, ENABLE_PIN, LIFT_MOTOR_CPR, LEDC_CHANNEL_0,LEDC_CHANNEL_1, LIFT_MOTOR_KP, LIFT_MOTOR_KI, LIFT_MOTOR_KD, 10);
    Central_Axis_Motor.init(CENTER_MOTOR_ENCODER_A, CENTER_MOTOR_ENCODER_B, CENTER_MOTOR_PWM_1, CENTER_MOTOR_PWM_2, ENABLE_PIN, CENTRAL_AXIS_CPR, LEDC_CHANNEL_2, LEDC_CHANNEL_3, CENTRAL_AXIS_KP, CENTRAL_AXIS_KI, CENTRAL_AXIS_KD, 10);
}

void loop() {
    // TODO: Implement safety checks that can pull the state into ESTOP (Current monitoring, diag pins, etc.)

    // TODO: Move state_switching logic to the callbacks (post and input are the only ones that should change to a state other than estop)

    //TODO: Add text to the LCD and set LEDs according to the state using the control panel class.
    switch (current_state) {
        case State::STATIONARY:
            Serial.println("Currently in STATIONARY state");
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