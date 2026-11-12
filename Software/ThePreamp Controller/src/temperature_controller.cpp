#include "temperature_controller.h"
#include "controller_config.h"
#include "debug.h"
#include <Adafruit_ADS1X15.h>
#include <Preferences.h>
#include <Wire.h>
#include <math.h>

static Adafruit_ADS1115 adc;
static bool adcReady = false;
static float measurements[TEMPERATURE_SENSOR_COUNT] = {NAN, NAN, NAN, NAN};
static uint32_t packedConfig = 0x2103; // A0/A1 enabled, in slots 1/2
static portMUX_TYPE configMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool configChanged = true;
static unsigned long lastMeasurement = 0;
static constexpr unsigned long refreshInterval = 10000;

static uint32_t packConfig(const TemperatureConfig &config)
{
  return config.enabledMask | (static_cast<uint32_t>(config.slots[0]) << 8) |
         (static_cast<uint32_t>(config.slots[1]) << 12);
}

static bool validConfig(const TemperatureConfig &config)
{
  return config.enabledMask <= 15 && config.slots[0] <= 4 && config.slots[1] <= 4 &&
         (config.slots[0] == 0 || config.slots[0] != config.slots[1]);
}

TemperatureConfig getTemperatureConfig()
{
  portENTER_CRITICAL(&configMux);
  const uint32_t packed = packedConfig;
  portEXIT_CRITICAL(&configMux);
  return {static_cast<uint8_t>(packed & 15), {static_cast<uint8_t>((packed >> 8) & 15),
                                               static_cast<uint8_t>((packed >> 12) & 15)}};
}

void loadTemperatureConfig()
{
  const TemperatureConfig defaults = {3, {static_cast<uint8_t>(Settings.DisplayTemperature1 ? 1 : 0),
                                          static_cast<uint8_t>(Settings.DisplayTemperature2 ? 2 : 0)}};
  portENTER_CRITICAL(&configMux);
  packedConfig = packConfig(defaults);
  portEXIT_CRITICAL(&configMux);
  Preferences prefs;
  if (!prefs.begin("temperature", false))
  {
    debugln("Unable to open temperature settings");
    return;
  }
  const uint32_t packed = prefs.getUInt("config", packedConfig);
  prefs.end();
  TemperatureConfig config = {static_cast<uint8_t>(packed & 15),
                              {static_cast<uint8_t>((packed >> 8) & 15),
                               static_cast<uint8_t>((packed >> 12) & 15)}};
  if (!validConfig(config) || (packed & ~0xff0fUL))
  {
    debugln("Invalid temperature settings; using defaults");
    return;
  }
  portENTER_CRITICAL(&configMux);
  packedConfig = packed;
  configChanged = true;
  portEXIT_CRITICAL(&configMux);
}

bool saveTemperatureConfig(const TemperatureConfig &config)
{
  if (!validConfig(config))
    return false;
  Preferences prefs;
  if (!prefs.begin("temperature", false))
  {
    debugln("Unable to open temperature settings for writing");
    return false;
  }
  const uint32_t packed = packConfig(config);
  const bool saved = prefs.putUInt("config", packed) == sizeof(packed);
  prefs.end();
  if (!saved)
  {
    debugln("Unable to save temperature settings");
    return false;
  }
  portENTER_CRITICAL(&configMux);
  const uint32_t previousConfig = packedConfig;
  packedConfig = packed;
  for (uint8_t channel = 0; channel < TEMPERATURE_SENSOR_COUNT; ++channel)
  {
    if (((packed ^ previousConfig) & (1 << channel)) != 0)
      measurements[channel] = NAN;
  }
  configChanged = true;
  portEXIT_CRITICAL(&configMux);
  return true;
}

void beginTemperatureSensors()
{
  adcReady = adc.begin();
  if (!adcReady)
  {
    debugln("ADS1115 not found; temperature readings unavailable");
    return;
  }
  adc.setGain(GAIN_ONE);
  adc.setDataRate(RATE_ADS1115_860SPS);
}

float getTemperature(uint8_t channel)
{
  constexpr float dividerVoltage = 3.3f;
  constexpr float referenceResistance = 10620.0f;
  constexpr uint8_t sampleCount = 8;
  const uint16_t mux[] = {ADS1X15_REG_CONFIG_MUX_SINGLE_0, ADS1X15_REG_CONFIG_MUX_SINGLE_1,
                          ADS1X15_REG_CONFIG_MUX_SINGLE_2, ADS1X15_REG_CONFIG_MUX_SINGLE_3};
  if (!adcReady || channel >= TEMPERATURE_SENSOR_COUNT)
    return NAN;

  int32_t adcSum = 0;
  for (uint8_t sample = 0; sample < sampleCount; ++sample)
  {
    Wire.beginTransmission(0x48);
    if (Wire.endTransmission() != 0)
    {
      debugln("ADS1115 disconnected; temperature readings unavailable");
      adcReady = false;
      return NAN;
    }
    adc.startADCReading(mux[channel], false);
    const unsigned long started = millis();
    while (!adc.conversionComplete())
    {
      if (millis() - started >= 100)
      {
        debugln("ADS1115 conversion timed out");
        adcReady = false;
        return NAN;
      }
      delay(1);
    }
    Wire.beginTransmission(0x48);
    if (Wire.endTransmission() != 0)
    {
      debugln("ADS1115 disconnected; temperature readings unavailable");
      adcReady = false;
      return NAN;
    }
    adcSum += adc.getLastConversionResults();
  }
  const float sensorVoltage = (static_cast<float>(adcSum) / sampleCount) * adc.computeVolts(1);
  if (sensorVoltage <= 0.0f || sensorVoltage >= dividerVoltage)
    return NAN;

  const float resistance = referenceResistance / (dividerVoltage / sensorVoltage - 1.0f);
  const float logRatio = logf(resistance / 4700.0f);
  // Vishay NTCALUG02A472G R/T calculator inverse polynomial, with R in ohms.
  constexpr float a1 = 0.003354016434680530f;
  constexpr float b1 = 0.000256523550896126f;
  constexpr float c1 = 0.000002605970120720520f;
  constexpr float d1 = 0.000000063292612648746f;
  const float inverseKelvin = a1 + logRatio * (b1 + logRatio * (c1 + logRatio * d1));
  if (!isfinite(inverseKelvin) || inverseKelvin <= 0.0f)
    return NAN;
  return constrain(1.0f / inverseKelvin - 273.15f, 0.0f, 99.0f);
}

bool updateTemperatureMeasurements(bool force)
{
  portENTER_CRITICAL(&configMux);
  const bool changed = configChanged;
  configChanged = false;
  portEXIT_CRITICAL(&configMux);
  if (!force && !changed && millis() - lastMeasurement < refreshInterval)
    return false;
  lastMeasurement = millis();
  const TemperatureConfig config = getTemperatureConfig();
  if (!adcReady && config.enabledMask)
    beginTemperatureSensors();
  float updated[TEMPERATURE_SENSOR_COUNT];
  for (uint8_t channel = 0; channel < TEMPERATURE_SENSOR_COUNT; ++channel)
    updated[channel] = (config.enabledMask & (1 << channel)) ? getTemperature(channel) : NAN;
  portENTER_CRITICAL(&configMux);
  if ((packedConfig & 15) == config.enabledMask)
    memcpy(measurements, updated, sizeof(measurements));
  portEXIT_CRITICAL(&configMux);
  return true;
}

float getCachedTemperature(uint8_t channel)
{
  if (channel >= TEMPERATURE_SENSOR_COUNT)
    return NAN;
  portENTER_CRITICAL(&configMux);
  const float value = measurements[channel];
  portEXIT_CRITICAL(&configMux);
  return value;
}

TemperatureSnapshot getTemperatureSnapshot()
{
  TemperatureSnapshot snapshot;
  portENTER_CRITICAL(&configMux);
  snapshot.enabledMask = packedConfig & 15;
  memcpy(snapshot.values, measurements, sizeof(snapshot.values));
  portEXIT_CRITICAL(&configMux);
  return snapshot;
}
