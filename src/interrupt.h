#ifndef INTERRUPT_H
#define INTERRUPT_H

#include <Arduino.h>
#include <Wire.h>

#define EXTADD 0x20
#define IODIR 0x00
#define SCL 22
#define SDA 21
#define OLAT 0x0A
#define GPIO 0x09
#define GPINTEN  0x02
#define GPPU 0x06
#define INTCAP 0x08
#define IPOL 0x00

extern volatile bool flag;
extern bool starting;
extern byte felix;

void ARDUINO_ISR_ATTR mybutton();
void init();
void start();
void response();
void led(byte hold);

#endif