#ifndef AUDIO_CONTROLLER_H
#define AUDIO_CONTROLLER_H

#include <Arduino.h>
#include "controller_config.h"

int calculateAttenuation(byte logicalStep, byte maxLogicalSteps, byte minAttenuation_dB, byte maxAttenuation_dB);
bool changeBalance();
boolean setInput(uint8_t NewInput);
void setPrevInput();
void setNextInput();
void setVolume(int16_t newVolumeStep);
void mute();
void unmute();
void unmuteOutput();
void muteOutput();

#endif
