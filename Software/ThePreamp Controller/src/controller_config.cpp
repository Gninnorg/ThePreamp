#include "controller_config.h"

mySettings Settings;
myRuntimeSettings RuntimeSettings;
extEEPROM eeprom(kbits_64, 1, 32);

void writeSettingsToEEPROM()
{
  eeprom.begin(extEEPROM::twiClock400kHz);
  eeprom.write(0, Settings.data, sizeof(Settings));
}

void readSettingsFromEEPROM()
{
  eeprom.begin(extEEPROM::twiClock400kHz);
  eeprom.read(0, Settings.data, sizeof(Settings));
}

void writeDefaultSettingsToEEPROM()
{
  setSettingsToDefault();
  writeSettingsToEEPROM();
  writeRuntimeSettingsToEEPROM();
}

void writeRuntimeSettingsToEEPROM()
{
  eeprom.begin(extEEPROM::twiClock400kHz);
  eeprom.write(sizeof(Settings) + 1, RuntimeSettings.data, sizeof(RuntimeSettings));
}

void readRuntimeSettingsFromEEPROM()
{
  eeprom.begin(extEEPROM::twiClock400kHz);
  eeprom.read(sizeof(Settings) + 1, RuntimeSettings.data, sizeof(RuntimeSettings));
}

void readUserSettingsFromEEPROM()
{
  eeprom.begin(extEEPROM::twiClock400kHz);
  eeprom.read(sizeof(Settings) + sizeof(RuntimeSettings) + 1, Settings.data, sizeof(Settings));
}

void writeUserSettingsToEEPROM()
{
  eeprom.begin(extEEPROM::twiClock400kHz);
  eeprom.write(sizeof(Settings) + sizeof(RuntimeSettings) + 1, Settings.data, sizeof(Settings));
}

void setSettingsToDefault()
{
  strcpy(Settings.ssid, "                                ");
  strcpy(Settings.pass, "                                ");
  strcpy(Settings.ip, "               ");
  strcpy(Settings.gateway, "               ");
  Settings.ExtPowerRelayTrigger = true;
  Settings.VolumeSteps = 60;
  Settings.MinAttenuation = 0;
  Settings.MaxAttenuation = 59;
  Settings.MaxStartVolume = Settings.VolumeSteps;
  Settings.MuteLevel = 0;
  Settings.RecallSetLevel = true;

#if IRCONF == 1
  Settings.IR_UP = 0x80840BF;
  Settings.IR_DOWN = 0x808E01F;
  Settings.IR_REPEAT = 0xFFFFFFFFFFFFFFFF;
  Settings.IR_LEFT = 0x808807F;
  Settings.IR_RIGHT = 0x808609F;
  Settings.IR_SELECT = 0x808AC53;
  Settings.IR_BACK = 0x80822DD;
  Settings.IR_MUTE = 0x80828D7;
  Settings.IR_PREVIOUS = 0x80818E7;
  Settings.IR_ON = 0x808926D;
  Settings.IR_OFF = 0x808926D;
  Settings.IR_1 = 0x808827D;
  Settings.IR_2 = 0x80842BD;
  Settings.IR_3 = 0x808E21D;
  Settings.IR_4 = 0x808CC33;
  Settings.IR_5 = 0x8082CD3;
#else
  Settings.IR_UP = 0x48AC40BF;
  Settings.IR_DOWN = 0x48AC609F;
  Settings.IR_REPEAT = 0xFFFFFFFFFFFFFFFF;
  Settings.IR_LEFT = 0x48ACC03F;
  Settings.IR_RIGHT = 0x48ACA05F;
  Settings.IR_SELECT = 0x48AC20DF;
  Settings.IR_BACK = 0x80822DD;
  Settings.IR_MUTE = 0x80828D7;
  Settings.IR_PREVIOUS = 0x80818E7;
  Settings.IR_ON = 0x48AC807F;
  Settings.IR_OFF = 0x48AC807F;
  Settings.IR_1 = 0x808827D;
  Settings.IR_2 = 0x80842BD;
  Settings.IR_3 = 0x808E21D;
  Settings.IR_4 = 0x808CC33;
  Settings.IR_5 = 0x8082CD3;
#endif

  Settings.Input[0].Active = INPUT_NORMAL;
  strcpy(Settings.Input[0].Name, "Input 1");
  Settings.Input[0].MaxVol = Settings.VolumeSteps;
  Settings.Input[0].MinVol = 1;
  Settings.Input[0].Gain = 0;
  Settings.Input[1].Active = INPUT_NORMAL;
  strcpy(Settings.Input[1].Name, "Input 2");
  Settings.Input[1].MaxVol = Settings.VolumeSteps;
  Settings.Input[1].MinVol = 1;
  Settings.Input[1].Gain = 0;
  Settings.Input[2].Active = INPUT_NORMAL;
  strcpy(Settings.Input[2].Name, "Input 3");
  Settings.Input[2].MaxVol = Settings.VolumeSteps;
  Settings.Input[2].MinVol = 1;
  Settings.Input[2].Gain = 0;
  Settings.Input[3].Active = INPUT_NORMAL;
  strcpy(Settings.Input[3].Name, "Input 4");
  Settings.Input[3].MaxVol = Settings.VolumeSteps;
  Settings.Input[3].MinVol = 1;
  Settings.Input[3].Gain = 0;
  Settings.Input[4].Active = INPUT_INACTIVATED;
  strcpy(Settings.Input[4].Name, "Input 5");
  Settings.Input[4].MaxVol = Settings.VolumeSteps;
  Settings.Input[4].MinVol = 1;
  Settings.Input[4].Gain = 0;
  Settings.Trigger1Active = 1;
  Settings.Trigger1Type = 0;
  Settings.Trigger1OnDelay = 0;
  Settings.Trigger1Temp = 0;
  Settings.Trigger2Active = 1;
  Settings.Trigger2Type = 0;
  Settings.Trigger2OnDelay = 0;
  Settings.Trigger2Temp = 0;
  Settings.TriggerInactOffTimer = 0;
  Settings.ScreenSaverActive = true;
  Settings.DisplayOnLevel = 3;
  Settings.DisplayDimLevel = 0;
  Settings.DisplayTimeout = 30;
  Settings.DisplayVolume = 1;
  Settings.DisplaySelectedInput = true;
  Settings.DisplayTemperature1 = 3;
  Settings.DisplayTemperature2 = 3;
  Settings.Version = VERSION;

  RuntimeSettings.CurrentInput = 0;
  RuntimeSettings.CurrentVolume = 0;
  RuntimeSettings.Muted = 0;
  RuntimeSettings.InputLastVol[0] = 0;
  RuntimeSettings.InputLastVol[1] = 0;
  RuntimeSettings.InputLastVol[2] = 0;
  RuntimeSettings.InputLastVol[3] = 0;
  RuntimeSettings.InputLastVol[4] = 0;
  RuntimeSettings.InputLastBal[0] = 127;
  RuntimeSettings.InputLastBal[1] = 127;
  RuntimeSettings.InputLastBal[2] = 127;
  RuntimeSettings.InputLastBal[3] = 127;
  RuntimeSettings.InputLastBal[4] = 127;
  RuntimeSettings.PrevSelectedInput = 0;
  RuntimeSettings.Version = VERSION;
}

float getTemperature(uint8_t pinNmbr)
{
  float Vin = 3.3;
  float Vout = 0;
  float Rref = 10000;
  float Rntc = 0;
  float Temp;

  int16_t adcValue = 15000;
  Vout = (adcValue * Vin) / 32767.0;

  Rntc = Rref * (Vin / Vout - 1);

  if (Rntc < 0)
    Temp = 0;
  else
    Temp = (-25.37 * log(Rntc)) + 239.43;

  if (Temp < 0)
    Temp = 0;
  else if (Temp > 65)
    Temp = 65;

  return Temp;
}

String exportSettingsAsJson() {
    StaticJsonDocument<2048> doc;

    doc["ssid"] = Settings.ssid;
    doc["pass"] = Settings.pass;
    doc["ip"] = Settings.ip;
    doc["gateway"] = Settings.gateway;

    doc["VolumeSteps"] = Settings.VolumeSteps;
    doc["MinAttenuation"] = Settings.MinAttenuation;
    doc["MaxAttenuation"] = Settings.MaxAttenuation;
    doc["MaxStartVolume"] = Settings.MaxStartVolume;
    doc["MuteLevel"] = Settings.MuteLevel;
    doc["RecallSetLevel"] = Settings.RecallSetLevel;

    doc["IR_ON"] = String(Settings.IR_ON);
    doc["IR_OFF"] = String(Settings.IR_OFF);
    doc["IR_UP"] = String(Settings.IR_UP);
    doc["IR_DOWN"] = String(Settings.IR_DOWN);
    doc["IR_REPEAT"] = String(Settings.IR_REPEAT);
    doc["IR_LEFT"] = String(Settings.IR_LEFT);
    doc["IR_RIGHT"] = String(Settings.IR_RIGHT);
    doc["IR_SELECT"] = String(Settings.IR_SELECT);
    doc["IR_BACK"] = String(Settings.IR_BACK);
    doc["IR_MUTE"] = String(Settings.IR_MUTE);
    doc["IR_PREVIOUS"] = String(Settings.IR_PREVIOUS);
    doc["IR_1"] = String(Settings.IR_1);
    doc["IR_2"] = String(Settings.IR_2);
    doc["IR_3"] = String(Settings.IR_3);
    doc["IR_4"] = String(Settings.IR_4);
    doc["IR_5"] = String(Settings.IR_5);

    JsonArray inputs = doc.createNestedArray("Input");
    for (int i = 0; i < 5; i++) {
        JsonObject input = inputs.createNestedObject();
        input["Active"] = Settings.Input[i].Active;
        input["Name"] = Settings.Input[i].Name;
        input["MaxVol"] = Settings.Input[i].MaxVol;
        input["MinVol"] = Settings.Input[i].MinVol;
        input["Gain"] = Settings.Input[i].Gain;
    }

    doc["ExtPowerRelayTrigger"] = Settings.ExtPowerRelayTrigger;
    doc["Trigger1Active"] = Settings.Trigger1Active;
    doc["Trigger1Type"] = Settings.Trigger1Type;
    doc["Trigger1OnDelay"] = Settings.Trigger1OnDelay;
    doc["Trigger1Temp"] = Settings.Trigger1Temp;
    doc["Trigger2Active"] = Settings.Trigger2Active;
    doc["Trigger2Type"] = Settings.Trigger2Type;
    doc["Trigger2OnDelay"] = Settings.Trigger2OnDelay;
    doc["Trigger2Temp"] = Settings.Trigger2Temp;
    doc["TriggerInactOffTimer"] = Settings.TriggerInactOffTimer;
    doc["ScreenSaverActive"] = Settings.ScreenSaverActive;
    doc["DisplayOnLevel"] = Settings.DisplayOnLevel;
    doc["DisplayDimLevel"] = Settings.DisplayDimLevel;
    doc["DisplayTimeout"] = Settings.DisplayTimeout;
    doc["DisplayVolume"] = Settings.DisplayVolume;
    doc["DisplaySelectedInput"] = Settings.DisplaySelectedInput;
    doc["DisplayTemperature1"] = Settings.DisplayTemperature1;
    doc["DisplayTemperature2"] = Settings.DisplayTemperature2;
    doc["Version"] = Settings.Version;

    String output;
    serializeJson(doc, output);
    return output;
}
