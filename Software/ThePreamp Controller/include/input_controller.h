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

// IR code learning state, used by the web UI to capture a new code for a command
extern volatile bool irLearnActive;
extern volatile bool irLearnCodeReady;
extern volatile uint64_t irLearnCode;

void setupRotaryEncoders();
byte getUserCommand();
void requestPowerToggle();

// Web handlers run on the AsyncTCP task and must never touch the SPI bus (displays, Muses), so they only
// queue what they want here (-1 = nothing pending) and loop() carries it out.
extern volatile int8_t requestedInput;
extern volatile int16_t requestedVolume;
extern volatile int8_t requestedMute; // -1 = none, 0 = unmute, 1 = mute
void processPendingRequests();
void startIrLearning();
void showInfoScreen();
void changeInfoPage(int direction);

#endif
