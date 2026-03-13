#include <Arduino.h>
#include <Wire.h>
#include "interrupt.h"

volatile bool flag;
bool starting;
byte felix;

void ARDUINO_ISR_ATTR mybutton(){
    flag = true;
}

void init(){
    flag = false;
    starting = true;
    felix = 0x00;
    
    Wire.begin(SDA, SCL);
    Serial.begin(115200);

    Wire.beginTransmission(EXTADD);
    Wire.write(IODIR);
    Wire.write(0x06);
    Wire.endTransmission();

    Wire.beginTransmission(EXTADD);
    Wire.write(GPINTEN);
    Wire.write(0x06);
    Wire.endTransmission();

    Wire.beginTransmission(EXTADD);
    Wire.write(GPPU);
    Wire.write(0x06);
    Wire.endTransmission();

    attachInterrupt(digitalPinToInterrupt(19), mybutton, FALLING);
}

void start(){
    Serial.println("Starting");
    Wire.beginTransmission(EXTADD);
    Wire.write(GPIO);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.beginTransmission(EXTADD);
    Wire.write(INTCAP); 
    Wire.endTransmission(false);
    Wire.requestFrom(EXTADD,1);
    while(Wire.available()) {
        byte c = Wire.read();    // Receive a byte as character
        Serial.println(c);         // Print the character
    }
    starting=false;            
}

void response(){
    Wire.beginTransmission(EXTADD);
    Wire.write(INTCAP);  
    Wire.endTransmission(false);
    Wire.requestFrom(EXTADD,1);
    Serial.println("Intcap:");
    Serial.println(INTCAP);
    while(Wire.available()) {
      felix = Wire.read();    // Receive a byte as character
    }
    led(felix);
    flag = !flag;
}
void led(byte hold){
    Serial.println("Got into loop");
    Wire.beginTransmission(EXTADD);
    Wire.write(GPIO);        
    Wire.write(((hold & (1 << 1)) >> 1) | (((hold >> 2) & 1) << 3));
    Wire.endTransmission();
    Serial.println(((hold & (1 << 1)) >> 1) | (((hold >> 2) & 1) << 3));

};