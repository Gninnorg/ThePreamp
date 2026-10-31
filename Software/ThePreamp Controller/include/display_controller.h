#ifndef DISPLAY_CONTROLLER_H
#define DISPLAY_CONTROLLER_H

#include <Arduino.h>
#include <SH1122.h>
#include <WiFi.h>
#include "controller_config.h"

extern SH1122 right_display;
extern SH1122 left_display;
extern bool ScreenSaverIsOn;
extern mySettings Settings;
extern myRuntimeSettings RuntimeSettings;

void left_display_update(void);
void right_display_update(void);
void drawSignalStrength(int);
void drawTemperatureMeasurements(void);
void displayBalance(byte Value); // Value 0-254, 127 is centered
void displayLogo(void);
bool isDisplayableName(const char *name); // true if every character exists in the input name font
// For each trigger: -1 = nothing to show, -2 = trigger has been turned on ("On"), >=0 = seconds remaining ("Wait... N")
void displayTriggerCountdown(int trigger1SecondsRemaining, int trigger2SecondsRemaining);
void displayInfoScreen(byte page); // page 0 = IP/QR code, page 1 = build dates
#define INFO_PAGE_COUNT 2
void ScreenSaverOn(void);
void ScreenSaverOff(void);

#endif
