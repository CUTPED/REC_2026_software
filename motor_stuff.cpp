#include <Arduino.h>
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "driver/pulse_cnt.h"
#include "driver/ledc.h"
 
// ─── Pin Definitions ─────────────────────────────────────────────────────────
#define ENC_PIN_A       4      // PCNT pulse pin  (channel A)
#define ENC_PIN_B       5      // PCNT control pin (channel B / direction)
#define MOTOR_PWM_PIN   16
#define MOTOR_DIR_PIN   17
 
// ─── Encoder Config ───────────────────────────────────────────────────────────
#define PCNT_UNIT       PCNT_UNIT_0
#define PCNT_H_LIM      30000   // Hardware counter high limit before overflow ISR fires
#define PCNT_L_LIM      -30000  // Hardware counter low limit
#define COUNTS_PER_REV   960  // PPR x 4 for full quadrature (240 PPR encoder)
 
// PCNT handles
pcnt_unit_handle_t pcnt_unit = NULL;
pcnt_channel_handle_t pcnt_chan0 = NULL;
pcnt_channel_handle_t pcnt_chan1 = NULL;
 
// ─── Pulse Counter Initialisation ──────────────────────────────────────────────────────
void initPCNT() {
  Serial.println("Start of iniitPCNT");
    // ── Unit ──────────────────────────────────────────────────────────────────
    pcnt_unit_config_t unit_cfg = {
        .low_limit   = PCNT_L_LIM,
        .high_limit  = PCNT_H_LIM,
         .flags = {
           .accum_count = true,        // driver accumulates overflows internally
         }
    };
    pcnt_new_unit(&unit_cfg, &pcnt_unit);
 
    // ── Add watch points at the limits so accum_count actually triggers ──────
    pcnt_unit_add_watch_point(pcnt_unit, PCNT_H_LIM);
    pcnt_unit_add_watch_point(pcnt_unit, PCNT_L_LIM);
 
    // ── Glitch filter ─────────────────────────────────────────────────────────
    pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = 1000,
    };
    pcnt_unit_set_glitch_filter(pcnt_unit, &filter_cfg);
 
    // ── Channel 0: A edge, B level ────────────────────────────────────────────
    pcnt_chan_config_t chan0_cfg = {
        .edge_gpio_num  = ENC_PIN_A,
        .level_gpio_num = ENC_PIN_B,
    };
    pcnt_new_channel(pcnt_unit, &chan0_cfg, &pcnt_chan0);
 
    pcnt_channel_set_edge_action(pcnt_chan0,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,   // A rising  → increment
        PCNT_CHANNEL_EDGE_ACTION_DECREASE);  // A falling → decrement
 
    pcnt_channel_set_level_action(pcnt_chan0,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,      // B high → keep direction
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);  // B low  → flip direction
 
    // ── Channel 1: B edge, A level ────────────────────────────────────────────
    pcnt_chan_config_t chan1_cfg = {
        .edge_gpio_num  = ENC_PIN_B,
        .level_gpio_num = ENC_PIN_A,
    };
    pcnt_new_channel(pcnt_unit, &chan1_cfg, &pcnt_chan1);
 
    pcnt_channel_set_edge_action(pcnt_chan1,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,   // B rising  → decrement
        PCNT_CHANNEL_EDGE_ACTION_INCREASE);  // B falling → increment
 
    pcnt_channel_set_level_action(pcnt_chan1,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
 
    // ── Enable and start ──────────────────────────────────────────────────────
    pcnt_unit_enable(pcnt_unit);
    pcnt_unit_clear_count(pcnt_unit);
    pcnt_unit_start(pcnt_unit);
}
 
 
// ─── LED Driver Initialization ────────────────────────────────────────────────────────
void initLEDC() {
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT, // Set PWM resolution 0-255
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 15000, // 15 kHz should be fine?
  };
  ledc_timer_config(&ledc_timer);
 
  ledc_channel_config_t ledc_channel = {
    .gpio_num = MOTOR_PWM_PIN,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .intr_type = LEDC_INTR_DISABLE, // not sure if this is the right one to use
    .timer_sel = LEDC_TIMER_0,
    .duty = 0, // Start with motor off
    .hpoint = 0, // Will need to set to something nonzero if using multiple motors/setting high and low
  };
  ledc_channel_config(&ledc_channel);
 
}
 
// ─── Felix's Code ──────────────────────────────────────────────────────
 
uint8_t x;
twai_node_handle_t twai_handle = NULL;
hw_timer_t *heartbeat_timer = NULL;
volatile uint8_t send_data[2] = {0,1};
twai_frame_t tx_msg = {
    .header ={
      .id = 0x10, // CAN message ID lower values are higher priority on the bus, so 0x10 is a relatively high priority message
      .ide = false, // This just means don't use extended frame format
    },
    .buffer = (uint8_t*)send_data, // Point to the data we want to send
    .buffer_len = sizeof(send_data), // This just specifies the length of the data we're sending, which is 1 byte in this case
};
 
volatile float rpm_value = 0.0f;
volatile bool rpm_flag = false;
volatile float raw_count = 0.0f;
int timestep = 20; // ms
 
// ─── Baby's First PID ──────────────────────────────────────────────────────
volatile float goal_motor_speed = 0.0f; // goes from 0 rpm to 135 rpm and back again
volatile float err = 0.0f; // error term for PID
volatile float Derr = 0.0f; // change in error for PID
volatile float PWM_1 = 0.0f; // PWM term 1
volatile float PWM_2 = 0.0f; // PWM term 2
float Kp = 0.5f;
float Ki = 5.0f;
float Kd = 70000000000.0f;
 
// ─── Where the stuff actually starts ──────────────────────────────────────────────────────
 
void IRAM_ATTR heartbeat_timer_callback(){
  memcpy(tx_msg.buffer, (uint8_t*)send_data, sizeof(send_data)); // Update the data field of the CAN message with the current value of x
  // ESP_ERROR_CHECK(twai_node_transmit(twai_handle, &tx_msg,0));
  static uint32_t rpmcounter = 0;
  rpmcounter++;
  if (rpmcounter %timestep == 0){
     static int32_t lastCount = 0;
     int currentCount = 0;
     pcnt_unit_get_count(pcnt_unit, &currentCount);
     int32_t countDiff = currentCount - lastCount;
     lastCount = currentCount;
     raw_count = currentCount;
     rpm_value = (((float)countDiff / COUNTS_PER_REV) / (timestep / 60000.0f)); // Just for testing, call the RPM calculation function every time we send a message to see the encoder value in the serial monitor. In a real application you might want to send the RPM value over CAN instead of just printing it.
     rpm_flag = true;
     rpmcounter = 0;
 
    // Goal Speed Function for testing, just ramps up and down between 0 and 135 rpm
    goal_motor_speed += 0.27;
    if (goal_motor_speed > 135.0f){
       goal_motor_speed = 135.0f;
      }
 
    // ─── PID ──────────────────────────────────────────────────────
     float prevErr = err; // store previous error for derivative term
     err = goal_motor_speed - rpm_value;
     Derr = err - prevErr; // change in error for PID
     PWM_1 += Kp * Derr + Ki * err * timestep/1000.0f; // P and I terms
     PWM_1 = PWM_1 < 0.0f ? 0.0f : (PWM_1 > 255.0f ? 255.0f : PWM_1); // Constrain PWM to valid range
     ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)PWM_1); //Set the new speed
     ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0); // Apply the new speed
  }
}
 
static bool IRAM_ATTR twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t* event_data, void* user_ctx){
  uint8_t rx_data[8];
  twai_frame_t rx_msg = {
    .buffer = rx_data,
    .buffer_len = sizeof(rx_data),
  };
  if(ESP_OK == twai_node_receive_from_isr(handle, &rx_msg)){
    Serial.printf("Received CAN message with ID: 0x%X, Data: ", rx_msg.header.id);
    for(int i = 0; i < rx_msg.buffer_len; i++){
      Serial.printf("%d ", rx_msg.buffer[i]);
    }
    Serial.println();
  }
 
  return true;
 
}
 
void setup() {
  //Serial setup for debugging
  Serial.begin(115200);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, PWM_2); // Currently set to low
  Serial.println(esp_get_idf_version());
  // Pulse counter setup for reading the encoder
  initPCNT();
  initLEDC();
  Serial.printf("pcnt_unit handle: %p\n", (void*)pcnt_unit);
  // CAN bus configuration
  twai_onchip_node_config_t twai_config = {
    .io_cfg ={
      .tx = GPIO_NUM_23, //Pin assignments for CAN TX and RX
      .rx = GPIO_NUM_22,
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
 
  heartbeat_timer = timerBegin(1000000); // Create a hardware timer with a prescaler of 80 (1 tick = 1 microsecond)
  timerAttachInterrupt(heartbeat_timer, heartbeat_timer_callback); // Attach the timer callback
  timerAlarm(heartbeat_timer, 1000, true, 0); // Set the timer to trigger every 1 millisecond (1,000 microseconds) and auto-reload
  timerStart(heartbeat_timer);
  x=1;
  Serial.println("End of setup");
}
 
void loop() {
  if (rpm_flag) {
    rpm_flag = false;
    Serial.print("goal: "); Serial.print(goal_motor_speed);
    Serial.print(" rpm: ");  Serial.print(rpm_value);
    Serial.print(" PWM: ");  Serial.println(PWM_1);
  }
}