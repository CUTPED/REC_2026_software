// #include <Arduino.h>
// #include "esp_twai.h"
// #include "esp_twai_onchip.h"
// #include "driver/pulse_cnt.h"
// #include "driver/ledc.h"

// #include "Motor_PID.h"
// #include "Control_Panel.h"

// // ─── Pin Definitions ─────────────────────────────────────────────────────────
// #define ENC_PIN_A       27      // PCNT pulse pin  (channel A)
// #define ENC_PIN_B       26      // PCNT control pin (channel B / direction)
// #define MOTOR_PWM_PIN   33
// #define MOTOR_DIR_PIN   32

// #define ENABLE_PIN 13

// ///
// #define MOTOR_1_ENCODER_A 4
// #define MOTOR_1_ENCODER_B 5
// #define MOTOR_2_ENCODER_A 18
// #define MOTOR_2_ENCODER_B 19
// #define MOTOR_3_ENCODER_A 26
// #define MOTOR_3_ENCODER_B 27

// #define MOTOR_1_PWM_1 16
// #define MOTOR_1_PWM_2 17
// #define MOTOR_2_PWM_1 23
// #define MOTOR_2_PWM_2 25
// #define MOTOR_3_PWM_1 32
// #define MOTOR_3_PWM_2 33
// ////
// #define MOTOR_1_CPR 960
// #define MOTOR_2_CPR 960
// #define MOTOR_3_CPR 960

// #define MOTOR_1_KP 0.5f
// #define MOTOR_1_KI 5.0f
// #define MOTOR_1_KD 83783.05f
// #define MOTOR_2_KP 0.5f
// #define MOTOR_2_KI 5.0f
// #define MOTOR_2_KD 83783.05f
// #define MOTOR_3_KP 0.5f
// #define MOTOR_3_KI 5.0f
// #define MOTOR_3_KD 83783.05f

// MotorPID Motor1;
// MotorPID Motor2;
// MotorPID Motor3;

// void loop() {
//   Motor1.update();
//   Motor2.update();
//   Motor3.update();

//   Serial.print("Motor 1 RPM: "); Serial.print(Motor1.getRPM());
//   Serial.print(" Motor 2 RPM: "); Serial.print(Motor2.getRPM());
//   Serial.print(" Motor 3 RPM: "); Serial.println(Motor3.getRPM());
//   delay(10);
// }

// void setup(){
//   Motor1.init(MOTOR_1_ENCODER_B, MOTOR_1_ENCODER_A, MOTOR_1_PWM_2, MOTOR_1_PWM_1, ENABLE_PIN, MOTOR_1_CPR, LEDC_CHANNEL_0, LEDC_CHANNEL_1, MOTOR_1_KP, MOTOR_1_KI, MOTOR_1_KD, 10);
//   Motor2.init(MOTOR_2_ENCODER_B, MOTOR_2_ENCODER_A, MOTOR_2_PWM_2, MOTOR_2_PWM_1, ENABLE_PIN, MOTOR_2_CPR, LEDC_CHANNEL_2, LEDC_CHANNEL_3, MOTOR_2_KP, MOTOR_2_KI, MOTOR_2_KD, 10);
//   Motor3.init(MOTOR_3_ENCODER_B, MOTOR_3_ENCODER_A, MOTOR_3_PWM_2, MOTOR_3_PWM_1, ENABLE_PIN, MOTOR_3_CPR, LEDC_CHANNEL_4, LEDC_CHANNEL_5, MOTOR_3_KP, MOTOR_3_KI, MOTOR_3_KD, 10);

//   Motor1.setGoalVelo(135.0f);
//   Motor2.setGoalVelo(135.0f);
//   Motor3.setGoalVelo(135.0f);

//   Motor1.enable();
//   Motor2.enable();
//   Motor3.enable();

// }


// ControlPanel controlPanel; 

// void IRAM_ATTR input_isr(uint16_t newState){
//     Serial.print("Input ISR triggered with new state: ");
//     Serial.println(newState, BIN);
// }

// void setup(){
//   Serial.begin(115200);

//   controlPanel.init();
//   controlPanel.setInputCallback(input_isr);

// }

// void loop() {

// }
// #include "MCP23008.h"
// #include <freertos/FreeRTOS.h>
// #include <freertos/semphr.h>
// MCP23008 mcp; // Create an instance of the MCP23008 class

// SemaphoreHandle_t i2cMutex; // Mutex for I2C access

// void callback(uint8_t newState){
//     Serial.print("Input ISR triggered with new state: ");
//     Serial.println(newState, BIN);
// }

// void setup() {
//   Serial.begin(115200);
//   i2cMutex = xSemaphoreCreateMutex();
//   mcp.init(i2cMutex, 17, 0x20); 
//   mcp.pinMode_stage(0, PIN_TYPE::PIN_PULLUP_INTERRUPT);
//   mcp.pinMode_stage(1, PIN_TYPE::PIN_OUTPUT);
//   mcp.pinMode_stage(2, PIN_TYPE::PIN_OUTPUT);
//   mcp.pinMode_stage(3, PIN_TYPE::PIN_OUTPUT);
//   mcp.pinMode_stage(4, PIN_TYPE::PIN_OUTPUT);
//   mcp.pinMode_stage(5, PIN_TYPE::PIN_OUTPUT);
//   mcp.pinMode_stage(6, PIN_TYPE::PIN_OUTPUT);
//   mcp.pinMode_stage(7, PIN_TYPE::PIN_OUTPUT);
//   mcp.commit(true); 
//   mcp.setUpdateCallback(callback); // Set the interrupt callback function

// }
// void loop() {

// }


#include <Arduino.h>
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "driver/pulse_cnt.h"
#include "driver/ledc.h"
 
#include "Motor_PID.h"
 
// ─── Pin Definitions ─────────────────────────────────────────────────────────
#define ENC_PIN_A       27      // PCNT pulse pin  (channel A)
#define ENC_PIN_B       26      // PCNT control pin (channel B / direction)
#define MOTOR_PWM_PIN   33
#define MOTOR_DIR_PIN   32
 
#define ENABLE_PIN 12
 
///
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
////
#define MOTOR_1_CPR 960
#define MOTOR_2_CPR 960
#define MOTOR_3_CPR 960
 
#define MOTOR_1_KP 0.5f
#define MOTOR_1_KI 5.0f
#define MOTOR_1_KD 83783.05f
#define MOTOR_2_KP 0.5f
#define MOTOR_2_KI 5.0f
#define MOTOR_2_KD 83783.05f
#define MOTOR_3_KP 0.5f
#define MOTOR_3_KI 5.0f
#define MOTOR_3_KD 83783.05f
 
#define LIFT_MOTOR_ENCODER_A 34
#define LIFT_MOTOR_ENCODER_B 35
#define LIFT_MOTOR_PWM_1 32
#define LIFT_MOTOR_PWM_2 33
 
#define LIFT_MOTOR_CPR 7974.4
#define LIFT_MOTOR_KP 0.005f
#define LIFT_MOTOR_KI 0.000f
#define LIFT_MOTOR_KD 0.00f

#define CENTER_MOTOR_ENCODER_A 36
#define CENTER_MOTOR_ENCODER_B 39
#define CENTER_MOTOR_PWM_1 19
#define CENTER_MOTOR_PWM_2 18

#define CENTER_MOTOR_CPR 1070
#define CENTER_MOTOR_KP 0.5f
#define CENTER_MOTOR_KI 5.0f
#define CENTER_MOTOR_KD 1000.0f

#define SHUTDOWN_PIN 4

// MotorPID Motor1;
// MotorPID Motor2;
// MotorPID Motor3;
//MotorPID LiftMotor;
//volatile float lift_motor_goal_position = 0.0f; // Target position for the lift motor in degrees /4
// MotorPID CenterMotor;
// volatile float center_motor_goal_speed = 50.0f;
#include "Control_Panel.h"
ControlPanel controlPanel;
// #include "MCP23008.h"

#include "Motor_PID.h"

MotorPID LiftMotor;

uint16_t x;
volatile bool flag_1 = false;
void IRAM_ATTR in_isr(uint16_t newState){
  x=newState;
  flag_1 = true;
}

// MCP23008 mcp; // Create an instance of the MCP23008 class
// SemaphoreHandle_t i2cMutex; // Mutex for I2C access

void loop() {

  // if (rpm_flag) {
  //   rpm_flag = false;
  //   Serial.print("goal: "); Serial.print(goal_motor_speed);
  //   Serial.print(" rpm: ");  Serial.print(rpm_value);
  //   Serial.print(" PWM: ");  Serial.println(PWM_1);
  // }
//   Motor1.update();
//   Motor2.update();
//   Motor3.update();
// CenterMotor.update();
//   LiftMotor.update();
//   LiftMotor.setGoalPos(lift_motor_goal_position);
  // CenterMotor.setGoalVelo(-60.0f);
  // Serial.print("Center Motor Speed: "); Serial.println(CenterMotor.getRPM());
  // Serial.print("PWM Output: "); Serial.println(CenterMotor._currentPWM);
 
//   analogWrite(LIFT_MOTOR_PWM_1, 50); Just for testing, set the PWM to a constant value to see the motor move. You should replace this with a call to the MotorPID class to set the PWM based on the PID output.
  if (flag_1){
    Serial.print("Input ISR triggered with new state: ");
    for (int i = 15; i >= 0; i--) {
      Serial.print(bitRead(x, i)); // Read and print the value of the i-th bit
    }
    Serial.println(); // Add a newline at the end
    flag_1 = false;
  }
  delay(10);
}
 
void setup(){
  pinMode(SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_PIN, HIGH); 
  Serial.begin(115200);
  controlPanel.init();
  controlPanel.setInputCallback(in_isr);
  liftMotor.init(LIFT_MOTOR_ENCODER_A, LIFT_MOTOR_ENCODER_B, LIFT_MOTOR_PWM_1, LIFT_MOTOR_PWM_2, ENABLE_PIN, LIFT_MOTOR_CPR, LEDC_CHANNEL_6, LEDC_CHANNEL_7, LIFT_MOTOR_KP, LIFT_MOTOR_KI, LIFT_MOTOR_KD, 10);
  liftMotor.enable();
  // i2cMutex = xSemaphoreCreateMutex();
  // mcp.init(i2cMutex, 17, 0x20); 
  // mcp.pinMode_stage(0, PIN_TYPE::PIN_OUTPUT);
  // mcp.pinMode_stage(1, PIN_TYPE::PIN_OUTPUT);
  // mcp.pinMode_stage(2, PIN_TYPE::PIN_OUTPUT);
  // mcp.pinMode_stage(3, PIN_TYPE::PIN_OUTPUT);
  // mcp.pinMode_stage(4, PIN_TYPE::PIN_PULLUP_INTERRUPT);
  // mcp.pinMode_stage(5, PIN_TYPE::PIN_PULLUP_INTERRUPT);
  // mcp.pinMode_stage(6, PIN_TYPE::PIN_PULLUP_INTERRUPT);
  // mcp.pinMode_stage(7, PIN_TYPE::PIN_PULLUP_INTERRUPT);
  // mcp.commit(true); 
  // mcp.setUpdateCallback(in_isr); // Set the interrupt callback function


//   Motor1.init(MOTOR_1_ENCODER_B, MOTOR_1_ENCODER_A, MOTOR_1_PWM_2, MOTOR_1_PWM_1, ENABLE_PIN, MOTOR_1_CPR, LEDC_CHANNEL_0, LEDC_CHANNEL_1, MOTOR_1_KP, MOTOR_1_KI, MOTOR_1_KD, 10);
//   Motor2.init(MOTOR_2_ENCODER_B, MOTOR_2_ENCODER_A, MOTOR_2_PWM_2, MOTOR_2_PWM_1, ENABLE_PIN, MOTOR_2_CPR, LEDC_CHANNEL_2, LEDC_CHANNEL_3, MOTOR_2_KP, MOTOR_2_KI, MOTOR_2_KD, 10);
//   Motor3.init(MOTOR_3_ENCODER_B, MOTOR_3_ENCODER_A, MOTOR_3_PWM_2, MOTOR_3_PWM_1, ENABLE_PIN, MOTOR_3_CPR, LEDC_CHANNEL_4, LEDC_CHANNEL_5, MOTOR_3_KP, MOTOR_3_KI, MOTOR_3_KD, 10);
//   LiftMotor.init(LIFT_MOTOR_ENCODER_A, LIFT_MOTOR_ENCODER_B, LIFT_MOTOR_PWM_1, LIFT_MOTOR_PWM_2, ENABLE_PIN, LIFT_MOTOR_CPR, LEDC_CHANNEL_6, LEDC_CHANNEL_7, LIFT_MOTOR_KP, LIFT_MOTOR_KI, LIFT_MOTOR_KD, 10);
  // CenterMotor.init(CENTER_MOTOR_ENCODER_B, CENTER_MOTOR_ENCODER_A, CENTER_MOTOR_PWM_2, CENTER_MOTOR_PWM_1, ENABLE_PIN, CENTER_MOTOR_CPR, LEDC_CHANNEL_6, LEDC_CHANNEL_7, CENTER_MOTOR_KP, CENTER_MOTOR_KI, CENTER_MOTOR_KD, 10);
//   pinMode(LIFT_MOTOR_PWM_1, OUTPUT);
//   pinMode(LIFT_MOTOR_PWM_2, OUTPUT);
//   digitalWrite(LIFT_MOTOR_PWM_2, LOW);
 
//   Motor1.setGoalVelo(135.0f);
//   Motor2.setGoaslVelo(135.0f);
//   Motor3.setGoalVelo(135.0f);
    // LiftMotor.setGoalPos(-100.0f); // Just for testing, set a goal position of 360 degrees (1 full rotation) for the lift motor
 
//   Motor1.enable();
//   Motor2.enable();
//   Motor3.enable();
//   LiftMotor.enable();
    //  CenterMotor.enable();
}
// void callback(uint8_t newState){
//     Serial.print("Input ISR triggered with new state: ");
//     Serial.println(newState, BIN);
// }

// void setup() {
//   Serial.begin(115200);



// // ─── Encoder Config ───────────────────────────────────────────────────────────
// #define PCNT_UNIT       PCNT_UNIT_0
// #define PCNT_H_LIM      30000   // Hardware counter high limit before overflow ISR fires
// #define PCNT_L_LIM      -30000  // Hardware counter low limit
// #define COUNTS_PER_REV   960  // PPR x 4 for full quadrature (240 PPR encoder)
 
// // PCNT handles
// pcnt_unit_handle_t pcnt_unit = NULL;
// pcnt_channel_handle_t pcnt_chan0 = NULL;
// pcnt_channel_handle_t pcnt_chan1 = NULL;
 
// // ─── Pulse Counter Initialisation ──────────────────────────────────────────────────────
// void initPCNT() {
//   Serial.println("Start of iniitPCNT");
//     // ── Unit ──────────────────────────────────────────────────────────────────
//     pcnt_unit_config_t unit_cfg = {
//         .low_limit   = PCNT_L_LIM,
//         .high_limit  = PCNT_H_LIM,
//          .flags = {
//            .accum_count = true,        // driver accumulates overflows internally
//          }
//     };
//     pcnt_new_unit(&unit_cfg, &pcnt_unit);
 
//     // ── Add watch points at the limits so accum_count actually triggers ──────
//     pcnt_unit_add_watch_point(pcnt_unit, PCNT_H_LIM);
//     pcnt_unit_add_watch_point(pcnt_unit, PCNT_L_LIM);
 
//     // ── Glitch filter ─────────────────────────────────────────────────────────
//     pcnt_glitch_filter_config_t filter_cfg = {
//         .max_glitch_ns = 1000,
//     };
//     pcnt_unit_set_glitch_filter(pcnt_unit, &filter_cfg);
 
//     // ── Channel 0: A edge, B level ────────────────────────────────────────────
//     pcnt_chan_config_t chan0_cfg = {
//         .edge_gpio_num  = ENC_PIN_A,
//         .level_gpio_num = ENC_PIN_B,
//     };
//     pcnt_new_channel(pcnt_unit, &chan0_cfg, &pcnt_chan0);
 
//     pcnt_channel_set_edge_action(pcnt_chan0,
//         PCNT_CHANNEL_EDGE_ACTION_INCREASE,   // A rising  → increment
//         PCNT_CHANNEL_EDGE_ACTION_DECREASE);  // A falling → decrement
 
//     pcnt_channel_set_level_action(pcnt_chan0,
//         PCNT_CHANNEL_LEVEL_ACTION_KEEP,      // B high → keep direction
//         PCNT_CHANNEL_LEVEL_ACTION_INVERSE);  // B low  → flip direction
 
//     // ── Channel 1: B edge, A level ────────────────────────────────────────────
//     pcnt_chan_config_t chan1_cfg = {
//         .edge_gpio_num  = ENC_PIN_B,
//         .level_gpio_num = ENC_PIN_A,
//     };
//     pcnt_new_channel(pcnt_unit, &chan1_cfg, &pcnt_chan1);
 
//     pcnt_channel_set_edge_action(pcnt_chan1,
//         PCNT_CHANNEL_EDGE_ACTION_DECREASE,   // B rising  → decrement
//         PCNT_CHANNEL_EDGE_ACTION_INCREASE);  // B falling → increment
 
//     pcnt_channel_set_level_action(pcnt_chan1,
//         PCNT_CHANNEL_LEVEL_ACTION_KEEP,
//         PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
 
//     // ── Enable and start ──────────────────────────────────────────────────────
//     pcnt_unit_enable(pcnt_unit);
//     pcnt_unit_clear_count(pcnt_unit);
//     pcnt_unit_start(pcnt_unit);
// }
 
 
// // ─── LED Driver Initialization ────────────────────────────────────────────────────────
// void initLEDC() {
//   ledc_timer_config_t ledc_timer = {
//     .speed_mode = LEDC_HIGH_SPEED_MODE,
//     .duty_resolution = LEDC_TIMER_8_BIT, // Set PWM resolution 0-255
//     .timer_num = LEDC_TIMER_0,
//     .freq_hz = 20000, // 20 kHz should be fine?
//   };
//   ledc_timer_config(&ledc_timer);
 
//   ledc_channel_config_t ledc_channel = {
//     .gpio_num = MOTOR_PWM_PIN,
//     .speed_mode = LEDC_HIGH_SPEED_MODE,
//     .channel = LEDC_CHANNEL_0,
//     .intr_type = LEDC_INTR_DISABLE, // not sure if this is the right one to use
//     .timer_sel = LEDC_TIMER_0,
//     .duty = 0, // Start with motor off
//     .hpoint = 0, // Will need to set to something nonzero if using multiple motors/setting high and low
//   };
//   ledc_channel_config(&ledc_channel);
 
// }
 
// // ─── Felix's Code ──────────────────────────────────────────────────────
 
// uint8_t x;
// twai_node_handle_t twai_handle = NULL;
// hw_timer_t *heartbeat_timer = NULL;
// volatile uint8_t send_data[2] = {0,1};
// twai_frame_t tx_msg = {
//     .header ={
//       .id = 0x10, // CAN message ID lower values are higher priority on the bus, so 0x10 is a relatively high priority message
//       .ide = false, // This just means don't use extended frame format
//     },
//     .buffer = (uint8_t*)send_data, // Point to the data we want to send
//     .buffer_len = sizeof(send_data), // This just specifies the length of the data we're sending, which is 1 byte in this case
// };
 
// volatile float rpm_value = 0.0f;
// volatile bool rpm_flag = false;
// volatile float raw_count = 0.0f;
// int timestep = 20; // ms
 
// // ─── Baby's First PID ──────────────────────────────────────────────────────
// volatile float goal_motor_speed = 0.0f; // goes from 0 rpm to 135 rpm and back again
// volatile float err = 0.0f; // error term for PID
// volatile float Derr = 0.0f; // change in error for PID
// volatile float PWM_1 = 0.0f; // PWM term 1
// volatile float PWM_2 = 0.0f; // PWM term 2
// float Kp = 0.5f;
// float Ki = 5.0f;
// float Kd = 70000000000.0f;
 
// // ─── Where the stuff actually starts ──────────────────────────────────────────────────────
 
// void IRAM_ATTR heartbeat_timer_callback(){
//   memcpy(tx_msg.buffer, (uint8_t*)send_data, sizeof(send_data)); // Update the data field of the CAN message with the current value of x
//   // ESP_ERROR_CHECK(twai_node_transmit(twai_handle, &tx_msg,0));
//   static uint32_t rpmcounter = 0;
//   rpmcounter++;
//   if (rpmcounter %timestep == 0){
//      static int32_t lastCount = 0;
//      int currentCount = 0;
//      pcnt_unit_get_count(pcnt_unit, &currentCount);
//      int32_t countDiff = currentCount - lastCount;
//      lastCount = currentCount;
//      raw_count = currentCount;
//      rpm_value = (((float)countDiff / COUNTS_PER_REV) / (timestep / 60000.0f)); // Just for testing, call the RPM calculation function every time we send a message to see the encoder value in the serial monitor. In a real application you might want to send the RPM value over CAN instead of just printing it.
//      rpm_flag = true;
//      rpmcounter = 0;
 
//     // Goal Speed Function for testing, just ramps up and down between 0 and 135 rpm
//     goal_motor_speed += 0.27;
//     if (goal_motor_speed > 135.0f){
//        goal_motor_speed = 135.0f;
//       }
 
//     // ─── PID ──────────────────────────────────────────────────────
//      float prevErr = err; // store previous error for derivative term
//      err = goal_motor_speed - rpm_value;
//      Derr = err - prevErr; // change in error for PID
//      PWM_1 += Kp * Derr + Ki * err * timestep/1000.0f; // P and I terms
//      PWM_1 = PWM_1 < 0.0f ? 0.0f : (PWM_1 > 255.0f ? 255.0f : PWM_1); // Constrain PWM to valid range
//      ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)PWM_1); //Set the new speed
//      ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0); // Apply the new speed
//   }
// }
 
// static bool IRAM_ATTR twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t* event_data, void* user_ctx){
//   uint8_t rx_data[8];
//   twai_frame_t rx_msg = {
//     .buffer = rx_data,
//     .buffer_len = sizeof(rx_data),
//   };
//   if(ESP_OK == twai_node_receive_from_isr(handle, &rx_msg)){
//     Serial.printf("Received CAN message with ID: 0x%X, Data: ", rx_msg.header.id);
//     for(int i = 0; i < rx_msg.buffer_len; i++){
//       Serial.printf("%d ", rx_msg.buffer[i]);
//     }
//     Serial.println();
//   }
 
//   return true;
 
// }
 
// void setup() {
//   //Serial setup for debugging
//   Serial.begin(115200);
 
//   pinMode(MOTOR_PWM_PIN, OUTPUT);
//   pinMode(MOTOR_DIR_PIN, OUTPUT);
 
 
//   digitalWrite(MOTOR_DIR_PIN, PWM_2); // Currently set to low
//   Serial.println(esp_get_idf_version());
//   // Pulse counter setup for reading the encoder
//   initPCNT();
//   initLEDC();
//   pinMode(ENABLE_PIN, OUTPUT);
//   digitalWrite(ENABLE_PIN, HIGH);
//   Serial.printf("pcnt_unit handle: %p\n", (void*)pcnt_unit);
//   // CAN bus configuration
//   twai_onchip_node_config_t twai_config = {
//     .io_cfg ={
//       .tx = GPIO_NUM_23, //Pin assignments for CAN TX and RX
//       .rx = GPIO_NUM_22,
//     },
//     .bit_timing = {
//       .bitrate = 500000, // Set CAN bus bitrate to 500 kbps
//     },
//     .tx_queue_depth = 5, // This isn't strictly necessary for basic operation, but it allows for buffering multiple messages if needed
//   };
//   twai_event_callbacks_t twai_callbacks = {
//     .on_rx_done = twai_rx_cb, // Call the twai_rx_cb function whenever a CAN message is received
//   };
 
//   //Start CAN Node
//   ESP_ERROR_CHECK(twai_new_node_onchip( &twai_config,&twai_handle));
//   ESP_ERROR_CHECK(twai_node_register_event_callbacks(twai_handle, &twai_callbacks,NULL));
//   ESP_ERROR_CHECK(twai_node_enable(twai_handle));
 
//   heartbeat_timer = timerBegin(1000000); // Create a hardware timer with a prescaler of 80 (1 tick = 1 microsecond)
//   timerAttachInterrupt(heartbeat_timer, heartbeat_timer_callback); // Attach the timer callback
//   timerAlarm(heartbeat_timer, 1000, true, 0); // Set the timer to trigger every 1 millisecond (1,000 microseconds) and auto-reload
//   timerStart(heartbeat_timer);
//   x=1;
//   Serial.println("End of setup");
// }
 