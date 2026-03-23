#include <Arduino.h>
#include "esp_twai.h"
#include "esp_twai_onchip.h"

uint8_t x;
twai_node_handle_t twai_handle = NULL;
hw_timer_t *heartbeat_timer = NULL;
volatile uint8_t send_data[2] = {0,1};
twai_node_handle_t twai_handle = NULL;
twai_frame_t tx_msg = {
    .header ={
      .id = 0x10, // CAN message ID lower values are higher priority on the bus, so 0x10 is a relatively high priority message
      .ide = false, // This just means don't use extended frame format 
    },
    .buffer = (uint8_t*)send_data, // Point to the data we want to send
    .buffer_len = sizeof(send_data), // This just specifies the length of the data we're sending, which is 1 byte in this case
};

void IRAM_ATTR heartbeat_timer_callback(){
  //Send the CAN message and wait for it to be sent before proceeding.
  memcpy(tx_msg.buffer, (uint8_t*)send_data, sizeof(send_data)); // Update the data field of the CAN message with the current value of x
  ESP_ERROR_CHECK(twai_node_transmit(twai_handle, &tx_msg,0));
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
}

void loop() {
  //This is a message once a second with an incrementing value in the data field.
  send_data[0] = x;
  send_data[1] = 1;
  x++;

  delay(100);
}
