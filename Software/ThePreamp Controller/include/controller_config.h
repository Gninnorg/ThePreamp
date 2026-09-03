#ifndef CONTROLLER_CONFIG_H
#define CONTROLLER_CONFIG_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <extEEPROM.h>
#include <Adafruit_MCP23008.h>
#include <Muses72323.h>

#ifndef POWER_CONTROL_PIN
#define POWER_CONTROL_PIN 2
#endif

#ifndef VERSION
#define VERSION (float)0.995
#endif

#ifndef IRCONF
#define IRCONF 1
#endif

#define EEPROM_Address 0x50

#define INPUT_HT_PASSTHROUGH 0
#define INPUT_NORMAL 1
#define INPUT_INACTIVATED 2

enum AppModeValues
{
  APP_NORMAL_MODE,
  APP_BALANCE_MODE,
  APP_STANDBY_MODE
};

enum UserInput
{
  KEY_NONE,
  KEY_UP,
  KEY_DOWN,
  KEY_REPEAT,
  KEY_SELECT,
  KEY_RIGHT,
  KEY_LEFT,
  KEY_BACK,
  KEY_1,
  KEY_2,
  KEY_3,
  KEY_4,
  KEY_5,
  KEY_MUTE,
  KEY_ON,
  KEY_OFF,
  KEY_PREVIOUS
};

struct InputSettings
{
  byte Active;
  char Name[8];
  byte MaxVol;
  byte MinVol;
  byte Gain;
};

typedef union
{
  struct
  {
    char ssid[33];
    char pass[33];
    char ip[16];
    char gateway[16];

    byte VolumeSteps;
    byte MinAttenuation;
    byte MaxAttenuation;
    byte MaxStartVolume;
    byte MuteLevel;
    byte RecallSetLevel;

    uint64_t IR_ON;
    uint64_t IR_OFF;
    uint64_t IR_UP;
    uint64_t IR_DOWN;
    uint64_t IR_REPEAT;
    uint64_t IR_LEFT;
    uint64_t IR_RIGHT;
    uint64_t IR_SELECT;
    uint64_t IR_BACK;
    uint64_t IR_MUTE;
    uint64_t IR_PREVIOUS;
    uint64_t IR_1;
    uint64_t IR_2;
    uint64_t IR_3;
    uint64_t IR_4;
    uint64_t IR_5;

    struct InputSettings Input[5];
    bool ExtPowerRelayTrigger;
    byte Trigger1Active;
    byte Trigger1Type;
    byte Trigger1OnDelay;
    byte Trigger1Temp;
    byte Trigger2Active;
    byte Trigger2Type;
    byte Trigger2OnDelay;
    byte Trigger2Temp;
    byte TriggerInactOffTimer;
    byte ScreenSaverActive;
    byte DisplayOnLevel;
    byte DisplayDimLevel;
    byte DisplayTimeout;
    byte DisplayVolume;
    byte DisplaySelectedInput;
    byte DisplayTemperature1;
    byte DisplayTemperature2;
    float Version;
  };
  byte data[318];
} mySettings;

typedef union
{
  struct
  {
    byte CurrentInput;
    byte CurrentVolume;
    bool Muted;
    byte InputLastVol[5];
    byte InputLastBal[5];
    byte PrevSelectedInput;
    float Version;
  };
  byte data[18];
} myRuntimeSettings;

extern mySettings Settings;
extern myRuntimeSettings RuntimeSettings;
extern extEEPROM eeprom;
extern Muses72323 muses;
extern Adafruit_MCP23008 relayController;

#define INPUT_HT_PASSTHROUGH 0
#define INPUT_NORMAL 1
#define INPUT_INACTIVATED 2

void setSettingsToDefault(void);
void writeSettingsToEEPROM();
void readSettingsFromEEPROM();
void writeDefaultSettingsToEEPROM();
void writeRuntimeSettingsToEEPROM();
void readRuntimeSettingsFromEEPROM();
void readUserSettingsFromEEPROM();
void writeUserSettingsToEEPROM();
String exportSettingsAsJson();
float getTemperature(uint8_t pinNmbr);

#endif
