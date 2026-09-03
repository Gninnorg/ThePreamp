#ifndef TRIGGER_CONTROLLER_H
#define TRIGGER_CONTROLLER_H

#include <Arduino.h>
#include <Adafruit_MCP23008.h>

extern Adafruit_MCP23008 relayController;

void setTrigger1On();
void setTrigger1Off();
void setTrigger2On();
void setTrigger2Off();

#endif
