#include <Arduino.h>

#define PWM_1 5
#define PWM_2 18
#define EN 17

uint8_t x = 0;

void setup() {
  
    Serial.begin(115200);
    pinMode(PWM_1, OUTPUT);
    pinMode(PWM_2, OUTPUT);
    pinMode(EN, OUTPUT);
    digitalWrite(EN, HIGH); // Enable the motor driver by setting the EN pin high
    digitalWrite(PWM_2, LOW); // Set the second PWM pin low to ensure the motor runs in the correct direction
}

void loop() {
    x+= 5;
    analogWrite(PWM_1, x);
    Serial.println(x); 
    delay(100);
}
