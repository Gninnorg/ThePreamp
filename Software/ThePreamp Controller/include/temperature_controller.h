#ifndef TEMPERATURE_CONTROLLER_H
#define TEMPERATURE_CONTROLLER_H

#include <Arduino.h>

constexpr uint8_t TEMPERATURE_SENSOR_COUNT = 4;
constexpr uint8_t TEMPERATURE_SLOT_COUNT = 2;

struct TemperatureConfig
{
  uint8_t enabledMask;
  uint8_t slots[TEMPERATURE_SLOT_COUNT]; // 0 = off, 1..4 = ADS1115 channels A0..A3
};

struct TemperatureSnapshot
{
  uint8_t enabledMask;
  float values[TEMPERATURE_SENSOR_COUNT];
};

void loadTemperatureConfig();
TemperatureConfig getTemperatureConfig();
bool saveTemperatureConfig(const TemperatureConfig &config);
void beginTemperatureSensors();
bool updateTemperatureMeasurements(bool force = false);
float getTemperature(uint8_t channel);
float getCachedTemperature(uint8_t channel);
TemperatureSnapshot getTemperatureSnapshot();

#endif
