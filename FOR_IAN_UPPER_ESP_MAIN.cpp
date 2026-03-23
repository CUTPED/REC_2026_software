#include <Arduino.h>
#include "Motor_PID.h"
#include <functional>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "driver/pulse_cnt.h"
#include "driver/ledc.h"


#define MAX_SECONDARY_AXIS_RPM 135.0f // Maximum secondary axis RPM 

//TODO: change these pin definitions for upper
// #define CENTER_MOTOR_ENCODER_A 36
// #define CENTER_MOTOR_ENCODER_B 39
// #define LIFT_MOTOR_ENCODER_A 34
// #define LIFT_MOTOR_ENCODER_B 35

// #define CENTER_MOTOR_PWM_1 19
// #define CENTER_MOTOR_PWM_2 18

// #define LIFT_MOTOR_PWM_1 32
// #define LIFT_MOTOR_PWM_2 33

// #define ENABLE_PIN 12

//TODO: change these CPR values for upper motors
// #define LIFT_MOTOR_CPR 7974.4
// #define CENTRAL_AXIS_CPR 7974.4


//TODO: change these PID tunings same for all upper motors
// #define LIFT_MOTOR_KP 0.5f
// #define LIFT_MOTOR_KI 0.1f
// #define LIFT_MOTOR_KD 0.05f



//State machine
enum class State: uint8_t {
    ESTOP = 5,
    POST = 1,
    STATIONARY = 2,
    NORMAL = 3,
    STOPPING = 4,
    MAINTENANCE = 6,
};

volatile State current_state = State::POST;


//TODO: (Felix) CAN timer and setup messages
hw_timer_t *heartbeat_timer = NULL;
twai_node_handle_t twai_handle = NULL;

// TODO: new motors same class x3
// MotorPID LiftMotor;
// MotorPID Central_Axis_Motor;

// This will be incremented in the heartbeat timer callback and set to 0 whenever a heartbeat is received from the ESP_H.
volatile uint8_t missed_heartbeats = 0; // If we go into the heartbeat isr and this value is 3 or more we go to ESTOP immediately (connection lost)


static bool IRAM_ATTR twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t* event_data, void* user_ctx){
    uint8_t rx_data[4];
  twai_frame_t rx_msg = {
    .buffer = rx_data,
    .buffer_len = sizeof(rx_data),
  };
  if(ESP_OK == twai_node_receive_from_isr(handle, &rx_msg)){
    //TODO: (Felix) Compare recieved state to current state and stop if they dont match dont reset the missed heartbeat counter
    if(rx_msg.header.id == 0x100){ // This is a heartbeat message
        if(static_cast<uint8_t>(current_state) == rx_msg.buffer[0]){ // If the state sent by the ESP_H doesn't match our current state, something is wrong, so go to ESTOP
            missed_heartbeats = 0;
        }
    }else{
        // TODO: (Felix) others I think this is just for startup?
    }
    // Serial.printf("Received CAN message with ID: 0x%X, Data: ", rx_msg.header.id);
    for(int i = 0; i < rx_msg.buffer_len; i++){
      Serial.printf("%d ", rx_msg.buffer[i]);
    }
    Serial.println();
  }
  return true;
}

//TODO: (Felix) Add Heartbeat ISR for Ian's stuff and sending CAN messages and incrementing missed_heartbeats (also have it trigger E-stop if we miss 3 heartbeats in a row)
uint8_t ERIN = 0;
void IRAM_ATTR heartbeat_timer_callback(){
    missed_heartbeats++;
    //TODO: Put the motor updates here (also rename ERIN unless you want her to be the heartbeat counter that ensures pid updates are 100 Hz) this really just needs to be motor.update() for each motor and reset ERIN you should do the PID position section of the Motor_PID class 

    if(missed_heartbeats >= 3){
        //ERROR_CODE = n // TODO: set an error code for missed heartbeats that can be sent over CAN and displayed on the LCD
        estop_isr(); // If we miss 3 heartbeats in a row, go to ESTOP
    }

}

void IRAM_ATTR estop_isr(){
    current_state = State::ESTOP; // Transition to ESTOP state immediately when the E-stop button is pressed
    // TODO: This should also pull enable low and send emergency CAN message
}

// RESET SECTION

// TODO: POST should be very simple for the upper esp (it boots into post state) it can run entirely in loop 
// like it should just repeatedly check its error pins while it waits to finish setup with the lower one


//TODO: Maintainence Mode functions theses should be short just make sure that when it gets a CAN message to move a motor it does that

void setup() {
    Serial.begin(115200);
    
    // TODO: Set up heartbeat timer

    // TODO: pinModes for diag, and ocm
    // TODO: Write DIAG interrupt for ESTOP and attach to (maybe just attach to estop_isr)
    // Reach goal would be to have it send a CAN message with a useful error code to report on the LCD like an error frame?

    // TODO: (Felix) Set up CAN with the appropriate callbacks and stuff

    //Motor Initialization
    //TODO: change these pin definitions for upper
    // LiftMotor.init(LIFT_MOTOR_ENCODER_A, LIFT_MOTOR_ENCODER_B, LIFT_MOTOR_PWM_1, LIFT_MOTOR_PWM_2, ENABLE_PIN, LIFT_MOTOR_CPR, LEDC_CHANNEL_0,LEDC_CHANNEL_1, LIFT_MOTOR_KP, LIFT_MOTOR_KI, LIFT_MOTOR_KD, 10);
    // Central_Axis_Motor.init(CENTER_MOTOR_ENCODER_A, CENTER_MOTOR_ENCODER_B, CENTER_MOTOR_PWM_1, CENTER_MOTOR_PWM_2, ENABLE_PIN, CENTRAL_AXIS_CPR, LEDC_CHANNEL_2, LEDC_CHANNEL_3, CENTRAL_AXIS_KP, CENTRAL_AXIS_KI, CENTRAL_AXIS_KD, 10);
}

void loop() {
    // TODO: Implement safety checks that can pull the state into ESTOP (IDK if there are any like global ones for this esp but if you think of any they can go here)

    // NOTE: All state switching comes from CAN so we can just use current_state

    switch (current_state) {
        case State::STATIONARY:
            //TODO: monitor pins to ensure motors are stationary (ESTOP if they start drawing current )
            //disable motors with disable function and set target speeds to 0

            //TODO: (Erin) Lights and stuff
            break;
        case State::NORMAL:
            // TODO: monitor pins OCM should get an upper bound and diag should be assumed ok since the interrupt from setup should take care of it

            //TODO: (Erin) Lights and stuff
            break;
        case State::ESTOP:
            // TODO: I am not sure what if anything we need here since this ESP has no power during ESTOP but there might be a cycle or something between transition to ESTOP and the actual power cut
            // So maybe call disable and pull all the pins low? 
            break;
        case State::MAINTENANCE:
            // TODO: This also needs some monitoring, updates to motor postions should be handeld in the heartbeat and CAN recieve callbacks
            break;
        //TODO: add other states (stopping is kinda weird we might just be able to put it above normal and not have the break so it just runs the normal code since it doesnt need to do anything different,
        // it still gets its targets from the CAN bus)
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