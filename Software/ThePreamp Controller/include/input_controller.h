#ifndef INPUT_CONTROLLER_H
#define INPUT_CONTROLLER_H

#include <Arduino.h>
#include <ClickEncoder.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include "controller_config.h"

extern byte appMode;
extern byte UIkey;
extern byte lastReceivedInput;
extern unsigned long last_KEY_ONOFF;

extern ClickEncoder *encoder1;
extern ClickEncoder *encoder2;
extern ClickEncoder::Button button1;
extern ClickEncoder::Button button2;
extern int16_t e1last, e1value;
extern int16_t e2last, e2value;
extern volatile int interruptCounter;
extern int totalInterruptCounter;
extern hw_timer_t *timer;
extern portMUX_TYPE timerMux;
extern IRrecv irrecv;
extern decode_results IRresults;

 byte getUserCommand();

#endif
