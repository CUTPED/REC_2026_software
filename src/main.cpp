#include <Arduino.h>
#include <Wire.h>
#include "MCP23008.h"
#include <functional>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

SemaphoreHandle_t i2cMutex;
MCP23008 *mcp = nullptr; 

void updateCallback(uint8_t newState) {
    Serial.print("MCP23008 Interrupt! New state: ");
    for (int i = 7; i >= 0; i--) Serial.print((newState >> i) & 1);
    Serial.println();
    if(!(newState & 0x02)){ // If the second bit is low, that means the button connected to that pin was pressed (assuming active-low with pull-up)
        mcp->write_stage(0, true); // Set the first bit high to turn on the LED connected to that pin
    }else{
      mcp->write_stage(0, false); // Set the first bit high to turn on the LED connected to that pin
    }
    if(!(newState & 0x04)){ // If the third bit is low, that means the button connected to that pin was pressed (assuming active-low with pull-up)
        mcp->write_stage(3, true); // Set the fourth bit high to turn on the LED connected to that pin
    }else{
        mcp->write_stage(3, false); // Set the fourth bit high to turn on the LED connected to that pin
    }
    mcp->commit(); // Send the staged changes over I2C
}

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA, SCL);
    i2cMutex = xSemaphoreCreateMutex();
    mcp = new MCP23008(i2cMutex); // Initialize the MCP23008 instance with the I2C mutex
    mcp->pinMode_stage(0, PIN_TYPE::PIN_OUTPUT);
    mcp->pinMode_stage(1, PIN_TYPE::PIN_PULLUP_INTERRUPT); 
    mcp->pinMode_stage(2, PIN_TYPE::PIN_PULLUP_INTERRUPT); 
    mcp->pinMode_stage(3, PIN_TYPE::PIN_OUTPUT);
    mcp->setUpdateCallback(updateCallback); // Set the interrupt callback function
    mcp->commit();
}

void loop(){
  Serial.println("Looping");
  vTaskDelay(1000 / portTICK_PERIOD_MS);  
}

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