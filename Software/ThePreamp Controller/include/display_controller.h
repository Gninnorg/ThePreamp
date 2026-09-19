#ifndef DISPLAY_CONTROLLER_H
#define DISPLAY_CONTROLLER_H

#include <Arduino.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include "controller_config.h"

extern U8G2_SH1122_256X64_F_4W_HW_SPI right_display;
extern U8G2_SH1122_256X64_F_4W_HW_SPI left_display;
extern bool ScreenSaverIsOn;
extern mySettings Settings;
extern myRuntimeSettings RuntimeSettings;

void left_display_update(void);
void right_display_update(void);
void drawSignalStrength(int);
void drawTemperatureMeasurements(void);
void displayBalance(byte Value); // Value 0-254, 127 is centered
void displayLogo(void);
// For each trigger: -1 = nothing to show, -2 = trigger has been turned on ("On"), >=0 = seconds remaining ("Wait... N")
void displayTriggerCountdown(int trigger1SecondsRemaining, int trigger2SecondsRemaining);
void ScreenSaverOn(void);
void ScreenSaverOff(void);

#endif
