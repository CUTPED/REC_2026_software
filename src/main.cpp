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
#define SHUTDOWN_PIN 4

#define LIFT_MOTOR_CPR 7974.4
#define CENTRAL_AXIS_CPR 7974.4

#define LIFT_MOTOR_KP 0.5f
#define LIFT_MOTOR_KI 0.1f
#define LIFT_MOTOR_KD 0.05f

#define CENTRAL_AXIS_KP 0.5f
#define CENTRAL_AXIS_KI 0.1f
#define CENTRAL_AXIS_KD 0.05f

#define CONTNUOUS_OPERATION true
#define STATION_TIME 30000 // Time to stay in stationary state before transitioning to normal mode in continuous operation mode


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
char text1[16];                                                                                                                                                                                       

// Timers will be created in setup heartbeat gets started in POST
// This is the ride cycle timer, started when transitioning from AWAITING_DISPATCH to NORMAL, and will be used to track the ride cycle and return to stop state
hw_timer_t *ride_cycle_timer = NULL;
// This is used for sending CAN messages (1KHz) and updating the PID controllers (100 Hz). 
hw_timer_t *heartbeat_timer = NULL;

TaskHandle_t POST_task_handle = NULL;

MotorPID LiftMotor;
MotorPID Central_Axis_Motor;

ControlPanel controlPanel; // Global instance of the control panel still need to call init in setup

//uint16_t error_code = 0; // This will be set to a non-zero value in the event of an error and can be sent over CAN and displayed on the LCD for diagnostics

// This will be incremented in the heartbeat timer callback and set to 0 whenever a heartbeat is received from the ESP_H.
uint8_t missed_heartbeats = 0; // If we go into the heartbeat isr and this value is 3 or more we go to ESTOP immediately (connection lost)

void IRAM_ATTR estop_isr(){
    current_state = State::ESTOP; // Transition to ESTOP state immediately when the E-stop button is pressed
    Central_Axis_Motor.disable();
    LiftMotor.disable();
    digitalWrite(SHUTDOWN_PIN, LOW); // Pull the shutdown pin low to the ride
}

volatile float secondary_motor_rpm_value = 0.0f; 

static bool IRAM_ATTR twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t* event_data, void* user_ctx){
    uint8_t rx_data[4];
  twai_frame_t rx_msg = {
    .buffer = rx_data,
    .buffer_len = sizeof(rx_data),
  };
  if(ESP_OK == twai_node_receive_from_isr(handle, &rx_msg)){    
    // ESP_H error
    if(rx_msg.header.id == 0x10){ 
        estop_isr();
        //error_code logic
    }
    // Heartbeat
    else if(rx_msg.header.id == 0x100){ // This is a heartbeat message
        if(static_cast<uint8_t>(current_state) == rx_msg.buffer[0]){ // If the state sent by the ESP_H doesn't match our current state, it might mean a transtion between heartbeats but if it happens 3 times its a problem
            missed_heartbeats = 0;
        }
    }
    // Setup
    else if(rx_msg.header.id == 0x50){ 
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(POST_task_handle, &higherPriorityTaskWoken);
    }
    // TODO: Maintenance mode messages 
    else if(rx_msg.header.id == 0x200){ 
        // Handle maintenance mode message idek what needs to be here
    }

    // for(int i = 0; i < rx_msg.buffer_len; i++){
    //   Serial.printf("%d ", rx_msg.buffer[i]);
    // }
    // Serial.println();
  }
  return true;
}

twai_node_handle_t twai_handle = NULL;
uint8_t heartbeat_data[4] = {0,0,0,0}; 
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

//TODO: add a way to compute the early_stop_ratio based on where the lift is when we press ride stop.
float early_stop_ratio = 1.0f; // This will be a value between 0 and 1 that represents how far through the ride cycle we are when we press the stop button, it can be used to scale down the target speeds/positions during the spin down phase to create a smoother stop if we stop early in the ride cycle
void spinDownCycle(unsigned long timer_value){ //early_stop_ratio represents where the motor was when we pressed ride stop.
    // This function will be called during the spin down phase of the ride cycle, it should ramp down the motors to 0 over the course of the spin down time
    float ramp_percentage = 1.0f - ((float)timer_value / SPIN_DOWN_TIME);
    LiftMotor.setGoalPos(ramp_percentage * MAX_LIFT_POS * early_stop_ratio);
    Central_Axis_Motor.setGoalVelo(ramp_percentage * MAX_CENTRAL_AXIS_RPM * early_stop_ratio);
    secondary_motor_rpm_value = ramp_percentage * MAX_SECONDARY_AXIS_RPM;
}

void rideCycleHandler(unsigned long timer_value) {
    if(timer_value <= SPIN_UP_TIME){
        spinUpCycle(timer_value);
    }else if(timer_value <= RIDE_CYCLE_TIME - SPIN_DOWN_TIME){
        normalRideCycle(timer_value - SPIN_UP_TIME);
    }
}

// RESET SECTION
void POST_task(void* pvParameters){
  while(true){
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // The callback for reset should notify this task
    current_state = State::POST;
    // return power
    digitalWrite(SHUTDOWN_PIN, HIGH); // Pull the shutdown pin high to supply power to the ride
    controlPanel.setDisplayText("POST","Powering Up");
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay 1s to allow ESP_H to boot
    controlPanel.setDisplayText("POST","Sending Setup");

    uint8_t setup_data[4] = {2,0,0,0}; 
    twai_frame_t setup_msg = {
        .header ={
        .id = 0x50, // CAN message ID lower values are higher priority on the bus
        .ide = false, // This just means don't use extended frame format 
        },
        .buffer = (uint8_t*)setup_data, // Point to the data we want to send
        .buffer_len = sizeof(setup_data), // This just specifies the length of the data we're sending, which is 1 byte in this case
    };
    ESP_ERROR_CHECK(twai_node_transmit(twai_handle, &setup_msg,0)); // Send the setup message to the ESP_H 
    if(ulTaskNotifyTake(pdTRUE, 2000 / portTICK_PERIOD_MS) == 0){ // The callback for reset should notify this task
        // If we don't receive a notification within 2 seconds, assume the ESP_H didn't boot properly and stay in ESTOP
        estop_isr();
        controlPanel.setDisplayText("ESTOP","No Handshake");
    }else{ 
        controlPanel.setDisplayText("POST","Final Checks");
        // Start the heartbeat properly
        missed_heartbeats = 0; // Reset missed heartbeats in case we were in ESTOP due to connection issues
        delayMicroseconds(500); //phase offset to avoid all the messages coming at once and overwhelming the bus
        heartbeat_timer_callback(); // Send a heartbeat immediately to let the ESP_H know we're alive and to reset the missed heartbeat counter on both sides
        timerStart(heartbeat_timer); // Start the heartbeat timer to begin sending heartbeats and monitoring the connection to the ESP_H

        //TODO: add the final list of motor checks we need (includeing ride off = 0)


        bool maintainence_mode = !(controlPanel.getState() & (1 << 14));
        bool normal_mode = !(controlPanel.getState() & (1 << 15));
        if(maintainence_mode && !normal_mode){
            current_state = State::MAINTENANCE;
        }else if(normal_mode && !maintainence_mode){  
            current_state = State::STATIONARY; 
        }else{
            estop_isr();
        }
    }
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

void IRAM_ATTR input_isr(uint16_t buttonData){
    // TODO: Move state_switching logic to the callbacks (post and input are the only ones that should change to a state other than estop)

    if(current_state == State::ESTOP){
        //do nothing since there is no escape outside of the reset callback defined elsewhere
        estop_isr(); // this is reduntant but you know its good in case athe shutdown pin is not set propererly or smth 
        return;
    }
    if(current_state == State::POST){
        //do nothing since the POST process handles its own state transitions. no buttons should affect that
        return;
    }
    if(!(buttonData & (1<<3))){ // E-stop button or other power loss
        estop_isr(); 
        return;
    }
    if(current_state == State::STATIONARY){
        if(!(buttonData & (1<<4))){ // Dispatch button 1
            if(!(buttonData & (1<<5 & 1<<7))){ // Dispatch lock and panel 2
                LiftMotor.enable();
                Central_Axis_Motor.enable();
                current_state = State::NORMAL;
                //restart the ride cycle timer
                timerRestart(ride_cycle_timer);
                timerAlarm(ride_cycle_timer, (RIDE_CYCLE_TIME-SPIN_DOWN_TIME) * 1000, false, 0); // Set the timer to trigger at the end of every ride cycle and not auto-reload
                timerStart(ride_cycle_timer); 
                //TODO: add text to LCD
            }else{
                //TODO: add text to LCD about rejecting dispatch due to lock or panel 2
            }
        }
        return;
    }
    if(current_state==State::NORMAL){
        if(!(buttonData & (1<<2))){ // Stop button
            early_stop_ratio = (float)LiftMotor.getPos() / MAX_LIFT_POS; // Compute the early stop ratio based on the current position of the lift when we stop the ride
            current_state = State::STOPPING;
        }
    }

    if(current_state==State::STOPPING){
        // Outside of ESTOP a transition from here is time based so use ride_event_isr
        return;
    }
    if(current_state == State::MAINTENANCE){
        //TODO: Later problem 
    }
}

void IRAM_ATTR ride_event_isr(){
    // This is the ISR for the ride event timer, it will handles transitions from running to stoped and stuff
    if(current_state == State::NORMAL){
        current_state = State::STOPPING;
        early_stop_ratio = (float)LiftMotor.getPos() / MAX_LIFT_POS; // Compute the early stop ratio based on the current position of the lift when we stop the ride 
        timerRestart(ride_cycle_timer);
        timerAlarm(ride_cycle_timer, SPIN_DOWN_TIME * 1000, false, 0); // Set the timer to trigger at the end of every ride cycle and not auto-re
        timerStart(ride_cycle_timer);
    }

    if(current_state == State::STOPPING){
        current_state = State::STATIONARY; // Transition to stationary state at the end of the ride cycle
        if(CONTNUOUS_OPERATION){
            timerRestart(ride_cycle_timer); 
            timerAlarm(ride_cycle_timer, STATION_TIME*1000, false, 0); // Set the timer to trigger after the station time to transition back to normal mode in continuous operation mode
            timerStart(ride_cycle_timer);
        }
    }

    if(current_state == State::STATIONARY){
        if(CONTNUOUS_OPERATION){
            LiftMotor.enable();
            Central_Axis_Motor.enable();
            current_state = State::NORMAL; // Transition back to normal state after the station time in continuous operation mode
            timerRestart(ride_cycle_timer); 
            timerAlarm(ride_cycle_timer, (RIDE_CYCLE_TIME-SPIN_DOWN_TIME) * 1000, false, 0); // Set the timer to trigger at the end of every ride cycle and not auto-reload
            timerStart(ride_cycle_timer);
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
    controlPanel.setInputCallback(input_isr); // Set the input callback to the input_isr function that will handle state switching based on button inputs
    strcpy(text1, "ESTOP");

    //ride_cycle_timer setup
    ride_cycle_timer = timerBegin(1000000); 
    timerAttachInterrupt(ride_cycle_timer, ride_event_isr); // Attach the timer callback 
    timerAlarm(ride_cycle_timer, (RIDE_CYCLE_TIME-SPIN_DOWN_TIME) * 1000, false, 0); // Set the timer to trigger at the end of every ride cycle and not auto-reload
    timerStop(ride_cycle_timer); // Start with the ride cycle timer stopped, it will be started when transitioning to NORMAL state

    heartbeat_timer = timerBegin(1000000); 
    timerAttachInterrupt(heartbeat_timer, heartbeat_timer_callback); // Attach the timer callback
    timerAlarm(heartbeat_timer, 1000, true, 0); // Set the timer to trigger every 1ms and auto-reload
    timerStop(heartbeat_timer); 

    //Motor Initialization
    LiftMotor.init(LIFT_MOTOR_ENCODER_A, LIFT_MOTOR_ENCODER_B, LIFT_MOTOR_PWM_1, LIFT_MOTOR_PWM_2, ENABLE_PIN, LIFT_MOTOR_CPR, LEDC_CHANNEL_0,LEDC_CHANNEL_1, LIFT_MOTOR_KP, LIFT_MOTOR_KI, LIFT_MOTOR_KD, 10);
    Central_Axis_Motor.init(CENTER_MOTOR_ENCODER_A, CENTER_MOTOR_ENCODER_B, CENTER_MOTOR_PWM_1, CENTER_MOTOR_PWM_2, ENABLE_PIN, CENTRAL_AXIS_CPR, LEDC_CHANNEL_2, LEDC_CHANNEL_3, CENTRAL_AXIS_KP, CENTRAL_AXIS_KI, CENTRAL_AXIS_KD, 10);
}

void loop() {
    // TODO: Implement safety checks that can pull the state into ESTOP (Current monitoring, diag pins, etc.)
    // TODO: Set LEDs according to the state using the control panel class.

    switch (current_state) {
        case State::STATIONARY:
            Serial.println("Currently in STATIONARY state");
            strcpy(text1, "STATIONARY");
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
            strcpy(text1, "NORMAL");
            // if(!rideCycleHandler(timerRead(ride_cycle_timer))){ // This checks if the ride cycle timer has reached the end of the ride cycle, and if so, it will transition back to STOP state. This is a non-blocking way to handle the ride cycle timing.
            //     current_state = State::STOP;
            //     Serial.println("Ride cycle ended, transitioning back to STOP state");
            // }
            break;
        case State::ESTOP:
            Serial.println("Currently in ESTOP state");
            strcpy(text1, "ESTOP");
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
            strcpy(text1, "MAINTENANCE");
            break;
    }
    controlPanel.setDisplayText(text1);
}
