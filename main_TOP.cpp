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

#define MOTOR_1_ENCODER_A 4
#define MOTOR_1_ENCODER_B 5
#define MOTOR_2_ENCODER_A 18
#define MOTOR_2_ENCODER_B 19
#define MOTOR_3_ENCODER_A 26
#define MOTOR_3_ENCODER_B 27

#define MOTOR_1_PWM_1 16
#define MOTOR_1_PWM_2 17
#define MOTOR_2_PWM_1 23
#define MOTOR_2_PWM_2 25
#define MOTOR_3_PWM_1 32
#define MOTOR_3_PWM_2 33

#define ENABLE_PIN 13

// #degine OCM_1 36
// #define OCM_2 39
// #define OCM_3 35

#define MOTOR_1_CPR 960
#define MOTOR_2_CPR 960
#define MOTOR_3_CPR 960

#define MOTOR_1_KP 0.5f
#define MOTOR_1_KI 0.1f
#define MOTOR_1_KD 0.05f
#define MOTOR_2_KP 0.5f
#define MOTOR_2_KI 0.1f
#define MOTOR_2_KD 0.05f
#define MOTOR_3_KP 0.5f
#define MOTOR_3_KI 0.1f
#define MOTOR_3_KD 0.05f



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

MotorPID Motor1;
MotorPID Motor2;
MotorPID Motor3;

void IRAM_ATTR estop_isr(){
    current_state = State::ESTOP; // Transition to ESTOP state immediately when the E-stop button is pressed
    Motor1.disable();
    Motor2.disable();
    Motor3.disable();
}

float secondary_motors_target = 0.0f;

// This will be incremented in the heartbeat timer callback and set to 0 whenever a heartbeat is received from the ESP_H.
volatile uint8_t missed_heartbeats = 0; // If we go into the heartbeat isr and this value is 3 or more we go to ESTOP immediately (connection lost)

static bool IRAM_ATTR twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t* event_data, void* user_ctx){
    uint8_t rx_data[4];
  twai_frame_t rx_msg = {
    .buffer = rx_data,
    .buffer_len = sizeof(rx_data),
  };
  if(ESP_OK == twai_node_receive_from_isr(handle, &rx_msg)){
    if(rx_msg.header.id == 0x10){ 
        estop_isr();
        //error_code logic
    }else if(rx_msg.header.id == 0x100){ // This is a heartbeat message
        current_state = static_cast<State>(rx_msg.buffer[0]); // Update our current state based on the heartbeat message from the ESP_H, this will help ensure we're in the correct state after a reset and can also be used to detect if we missed a state transition
        if (current_state == State::NORMAL){
            Motor1.enable();
            Motor2.enable();
            Motor3.enable();
        }
        secondary_motors_target = (float)rx_msg.buffer[1]; // Update the target speed for the secondary motors based on the heartbeat message from the ESP_H, this is just an example of how we can send additional data in the heartbeat message and use it to control the motors
        Motor1.setGoalVelo(secondary_motors_target);
        Motor2.setGoalVelo(secondary_motors_target);
        Motor3.setGoalVelo(secondary_motors_target);
        
    }else if(rx_msg.header.id == 0x50){
        current_state = State::POST; 
        uint8_t setup_data[4] = {2,0,0,0}; 
        twai_frame_t setup_msg = {
            .header ={
            .id = 0x50, // CAN message ID lower values are higher priority on the bus
            .ide = false, // This just means don't use extended frame format 
            },
            .buffer = (uint8_t*)setup_data, // Point to the data we want to send
            .buffer_len = sizeof(setup_data), // This just specifies the length of the data we're sending, which is 1 byte in this case
        };
        ESP_ERROR_CHECK(twai_node_transmit(twai_handle, &setup_msg, 0));
        timerStart(heartbeat_timer); // Start the heartbeat timer to begin sending heartbeats and monitoring the connection to the ESP_H, we start it here because we want to wait until we receive the setup message back from the ESP_H before we start monitoring the connection
    }
    // for(int i = 0; i < rx_msg.buffer_len; i++){
    //   //Serial.printf("%d ", rx_msg.buffer[i]);
    // }
    // //Serial.println();
  }
  return true;
}

//TODO: (Felix) Add Heartbeat ISR for Ian's stuff and sending CAN messages and incrementing missed_heartbeats (also have it trigger E-stop if we miss 3 heartbeats in a row)

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
        Motor1.update();
        Motor2.update();
        Motor3.update();
        pid_update_counter = 0;
    }

    heartbeat_data[0] = static_cast<uint8_t>(current_state); // Send the current state in the heartbeat message, you can also add other data here if needed
    heartbeat_data[1] = (uint8_t)secondary_motors_target; 
    heartbeat_data[2] = 0; // These bits will be used in maintainence mode
    heartbeat_data[3] = 0; 
    memcpy(heartbeat_msg.buffer, (uint8_t*)heartbeat_data, sizeof(heartbeat_data)); // Update the data field of the CAN message with the current value of x
    ESP_ERROR_CHECK(twai_node_transmit(twai_handle, &heartbeat_msg,0));

    pid_update_counter++;
}

//TODO: Maintainence Mode functions theses should be short just make sure that when it gets a CAN message to move a motor it does that

void setup() {
    //Serial.begin(115200);
    heartbeat_timer = timerBegin(1000000); 
    timerAttachInterrupt(heartbeat_timer, heartbeat_timer_callback); // Attach the timer callback
    timerAlarm(heartbeat_timer, 1000, true, 0); // Set the timer to trigger every 1ms and auto-reload
    timerStop(heartbeat_timer); 
    // TODO: pinModes for diag, and ocm

    // TODO: Write DIAG interrupt for ESTOP and attach to (maybe just attach to estop_isr)
    // Reach goal would be to have it send a CAN message with a useful error code to report on the LCD like an error frame?

    //Motor Initialization
    Motor1.init(MOTOR_1_ENCODER_A, MOTOR_1_ENCODER_B, MOTOR_1_PWM_1, MOTOR_1_PWM_2, ENABLE_PIN, MOTOR_1_CPR, LEDC_CHANNEL_0, LEDC_CHANNEL_1, MOTOR_1_KP, MOTOR_1_KI, MOTOR_1_KD, 10);
    Motor2.init(MOTOR_2_ENCODER_A, MOTOR_2_ENCODER_B, MOTOR_2_PWM_1, MOTOR_2_PWM_2, ENABLE_PIN, MOTOR_2_CPR, LEDC_CHANNEL_2, LEDC_CHANNEL_3, MOTOR_2_KP, MOTOR_2_KI, MOTOR_2_KD, 10);
    Motor3.init(MOTOR_3_ENCODER_A, MOTOR_3_ENCODER_B, MOTOR_3_PWM_1, MOTOR_3_PWM_2, ENABLE_PIN, MOTOR_3_CPR, LEDC_CHANNEL_4, LEDC_CHANNEL_5, MOTOR_3_KP, MOTOR_3_KI, MOTOR_3_KD, 10);
    twai_onchip_node_config_t twai_config = {
    .io_cfg ={
      .tx = GPIO_NUM_1, //Pin assignments for CAN TX and RX
      .rx = GPIO_NUM_3,
    },
    .bit_timing = {
      .bitrate = 500000, // Set CAN bus bitrate to 500 kbps
    }, 
    .tx_queue_depth = 5, // This isn't strictly necessary for basic operation, but it allows for buffering multiple messages if needed
  };
  twai_event_callbacks_t twai_callbacks = {
    .on_rx_done = twai_rx_cb, // Call the twai_rx_cb function whenever a CAN message is received
  };

  //Start CAN Node
  ESP_ERROR_CHECK(twai_new_node_onchip( &twai_config,&twai_handle));
  ESP_ERROR_CHECK(twai_node_register_event_callbacks(twai_handle, &twai_callbacks,NULL));
  ESP_ERROR_CHECK(twai_node_enable(twai_handle));

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
        case State::STOPPING:
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
//     //Serial.print("MCP23008 Interrupt! New state: ");
//     for (int i = 7; i >= 0; i--) //Serial.print((newState >> i) & 1);
//     //Serial.println();
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
//     //Serial.begin(115200);
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
//   //Serial.println("Looping");
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
  //Serial.begin(115200);
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
  //Serial.println("1");
  delay(1000);
  Wire.beginTransmission(EXTADD);
  Wire.write(GPIO);
  Wire.write(0x00);
  Wire.endTransmission();
  //Serial.println("0");
  delay(1000);
}*/