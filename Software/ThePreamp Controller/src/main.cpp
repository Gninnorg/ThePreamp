/*
**
**    Controller for ThePreAmp
**
**    Copyright (c) 2024 Carsten Grønning, Jan Abkjer Tofft
**
**
**   Todo
**   - DONE - check output relay
**   - clean up
**   - DONE - add discrete on/off
**   - DONE - add support for learning IR codes 
**   - Add support for balance control
**   - DONE - Add support for gain control
**   - Add support for temperature display
**   - Add support for MQTT
**   - Add UI for settings
**   - Shrink Elegant OTA - Remove personalization
**   - Add trigger control at startup - around line 780
**
*/


#define VERSION (float)0.995
// IRCONF == 1 Jan 
// IRCONF == 0 Carsten
// Remember to change VERSION to update eprom

#define IRCONF 1


#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP23008.h>
#include <extEEPROM.h>
#include <ClickEncoder.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <Muses72323.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <DNSServer.h>
#include "SPIFFS.h"
#include <AsyncTCP.h>
#include <WebSerial.h>
#include <ArduinoJson.h>
#include "logo.h"
#include "wifi_QR.h"
#include "controller_config.h"
#define DEBUG_USE_WEBSERIAL
#include "debug.h"
#include "display_controller.h"
#include "wifi_support.h"
#include "audio_controller.h"
#include "input_controller.h"
#include "trigger_controller.h"

#undef minimum
#ifndef minimum
#define minimum(a, b) ((a) < (b) ? (a) : (b))
#endif

// Webserver
AsyncWebServer server(80);
DNSServer dnsServer;

// Search for parameter in HTTP POST request - used for wifi configuration page
const char *PARAM_INPUT_1 = "ssid";
const char *PARAM_INPUT_2 = "pass";
const char *PARAM_INPUT_3 = "ip";
const char *PARAM_INPUT_4 = "gateway";

IPAddress localIP;
// IPAddress localIP(192, 168, 1, 200); // hardcoded

// Set your Gateway IP address
IPAddress localGateway;
// IPAddress localGateway(192, 168, 1, 1); //hardcoded
IPAddress subnet(255, 255, 0, 0);

// Timer variables
unsigned long previousMillis = 0;
extern const long interval = 10000; // interval to wait for Wi-Fi connection (milliseconds)

/* ----- Hardware SPI -----
  GND    ->    GND
  VCC    ->    3V3
  SCK    ->    D18 (SCL on displays, CLOCK on Muses)
  MISO   ->    D19 no planned use in ThePreAmp but pin is made available for potential use on controller and analog board
  MOSI   ->    D23 (SDA on displays, DATA on Muses)
  CS     ->    D5 (LATCH on Muses), 
               D12 (CS on display 1), 
               D13 (CS on display 2)
  Additional signals required by displays:
    DC     ->    D4 (DC on displays),
                 not used (Muses)
    RST    ->    D32 (RST display 1), 
                 D33 (RST display 2), 
                 not used (Muses)
*/

#define SPI_SCK_PIN 18
#define SPI_MISO_PIN 19
#define SPI_CS_MUSES_PIN 5
#define SPI_CS_RIGHT_DISPLAY_PIN 12
#define SPI_CS_LEFT_DISPLAY_PIN 13
#define SPI_DC_BOTH_DISPLAYS_PIN 4
#define SPI_RST_RIGHT_DISPLAY_PIN 32
#define SPI_RST_LEFT_DISPLAY_PIN 33

U8G2_SH1122_256X64_F_4W_HW_SPI right_display(U8G2_R0, SPI_CS_RIGHT_DISPLAY_PIN, SPI_DC_BOTH_DISPLAYS_PIN, SPI_RST_RIGHT_DISPLAY_PIN);
U8G2_SH1122_256X64_F_4W_HW_SPI left_display(U8G2_R0, SPI_CS_LEFT_DISPLAY_PIN, SPI_DC_BOTH_DISPLAYS_PIN, SPI_RST_LEFT_DISPLAY_PIN);

/* ----- I2C -----
GND    ->    GND
VCC    ->    3V3
SCL    ->    D22
SDA    ->    D21
*/
#define I2C_SCL_PIN 22 // ESP32 standard pin for SCL
#define I2C_SDA_PIN 21 // ESP32 standard pin for SDA

Adafruit_ADS1115 ads1115;

#define IR_RECEIVER_INPUT_PIN 15
IRrecv irrecv(IR_RECEIVER_INPUT_PIN);

decode_results IRresults;

// ----- OTHER PIN DEFINITIONS ---- 
unsigned long mil_On = millis(); // Holds the millis from last power on (or restart)
bool ScreenSaverIsOn = false; // Used to indicate whether the screen saver is running or not
unsigned long mil_LastUserInput = millis(); // Used to keep track of the time of the last user interaction (part of the screen saver timing)
unsigned long mil_onRefreshTemperatureDisplay; // Used to time how often the display of temperatures is updated

// Update intervals for the display/notification of temperatures
#define TEMP_REFRESH_INTERVAL 10000         // Interval while on
#define TEMP_REFRESH_INTERVAL_STANDBY 60000 // Interval while in standby

byte appMode = APP_NORMAL_MODE;

byte UIkey; // holds the last received user input (from rotary encoders or IR)
byte lastReceivedInput = KEY_NONE;
unsigned long last_KEY_ONOFF = millis(); // Used to ensure that fast repetition of KEY_ONOFF is not accepted

// Shared controller settings and runtime state are defined in controller_config.cpp.

// Setup Muses72323 -----------------------------------------------------------
Muses72323 muses(0, SPI_CS_MUSES_PIN); // Run at 500kHz

// Setup Relay Controller------------------------------------------------------
Adafruit_MCP23008 relayController;

// Function declarations
void toAppNormalMode();
void toStandbyMode();
void setup();
void startUp();
void loop();

void setup() {
  // Serial port for debugging purposes
  #if DEBUG == 1
    Serial.begin(115200);
  #endif
  
  //delay(5000); // Allow power supply to stabilize before starting up the controller
  SPI.begin();
  Wire.begin();

  right_display.setBusClock(4000000);
  right_display.begin();
  right_display.setFont(u8g2_font_inb63_mn); 
  
  left_display.setBusClock(4000000);
  left_display.begin();
  left_display.setFont(u8g2_font_inb63_mn);
  
  setupRotaryEncoders();
  
  relayController.begin();
  // Define all pins as OUTPUT and disable all relays
  for (byte pin = 0; pin <= 7; pin++)
  {
    relayController.pinMode(pin, OUTPUT);
    relayController.digitalWrite(pin, LOW);
  }

  ads1115.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  ads1115.begin();
  
  // Start IR reader
  irrecv.enableIRIn();

  // Read setting from EEPROM
  readSettingsFromEEPROM();
  readRuntimeSettingsFromEEPROM();

  // Check if settings stored in EEPROM are INVALID - if so, we write the default settings to the EEPROM and continue with those
  if ((Settings.Version != (float)VERSION) || (RuntimeSettings.Version != (float)VERSION))
  {
    debugln("Eeprom settings are invalid - writing default settings to EEPROM");
    debug("Settings.Version: "); debug(Settings.Version); debug(" != "); debugln((float)VERSION);
    debug("RuntimeSettings.Version: "); debug(RuntimeSettings.Version); debug(" != "); debugln((float)VERSION);
    right_display.clearBuffer();
    right_display.drawStr(0, 63, "Reset");
    right_display.sendBuffer();
    delay(2000);
    writeDefaultSettingsToEEPROM();
  }
  else
  {
    debugln("Eeprom settings are ok");
    debug("Settings.Version: "); debug(Settings.Version); debug(" = "); debugln((float)VERSION);
    debug("RuntimeSettings.Version: "); debug(RuntimeSettings.Version); debug(" = "); debugln((float)VERSION);
  }
  
  startWiFiSupport();

  // Set pin mode for control of power relay
  pinMode(POWER_CONTROL_PIN, OUTPUT);

  muses.begin();
  muses.setExternalClock(false);
  muses.setZeroCrossingOn(true);
    
  startUp();
}

void startUp()
{
  debugln("Starting up...");
  displayLogo();

  if(WiFi.status() != WL_CONNECTED)
  {
     initWiFi();
  }
 
  // Turn on external circuit via optocoupler
  if (Settings.ExtPowerRelayTrigger)
  {
    digitalWrite(POWER_CONTROL_PIN, HIGH);
  }
  
  // The controller is now ready - save the timestamp
  mil_On = millis();

  /*
  // If triggers are active then wait for the set number of seconds and turn them on
  unsigned long delayTrigger1 = (Settings.Trigger1Active) ? (mil_On + Settings.Trigger1OnDelay * 1000) : 0;
  unsigned long delayTrigger2 = (Settings.Trigger2Active) ? (mil_On + Settings.Trigger2OnDelay * 1000) : 0;

  while (delayTrigger1 || delayTrigger2)
  {
    if (millis() > delayTrigger1 && delayTrigger1 != 0)
    {
      setTrigger1On();
      delayTrigger1 = 0;
      // oled.print3x3Number(2, 1, 0, false);
    }
    else
    {
      //if (Settings.Trigger1Active && delayTrigger1 != 0)
        // oled.print3x3Number(2, 1, (delayTrigger1 - millis()) / 1000, false);
    }

    if (millis() > delayTrigger2 && delayTrigger2 != 0)
    {
      setTrigger2On();
      delayTrigger2 = 0;
      // oled.print3x3Number(11, 1, 0, false);
    }
    else
    {
      //if (Settings.Trigger2Active && delayTrigger2 != 0)
      //  oled.print3x3Number(11, 1, (delayTrigger2 - millis()) / 1000, false);
    }
  }
  // oled.clear();
  */

  ScreenSaverOff();
  appMode = APP_NORMAL_MODE;
  
  // Keep start volume for current input lower than max allowed start volume
  RuntimeSettings.InputLastVol[RuntimeSettings.CurrentInput] = minimum(RuntimeSettings.InputLastVol[RuntimeSettings.CurrentInput], Settings.MaxStartVolume); // Avoid setting volume higher than MaxStartVol
  setInput(RuntimeSettings.CurrentInput);

  // Enable output / trigger output relay
  unmuteOutput();

  left_display_update();
  right_display_update();

  UIkey = KEY_NONE;
  lastReceivedInput = KEY_NONE;
}

void loop()
{
  ElegantOTA.loop();
  WebSerial.loop();
  
  UIkey = getUserCommand();

  switch (appMode)
  {
  case APP_NORMAL_MODE:
    /*- TO DO
        if (millis() > mil_onRefreshTemperatureDisplay + TEMP_REFRESH_INTERVAL)
        {
          displayTemperatures();
          notifyClients(getJSONTempValues());
          if (((Settings.Trigger1Temp != 0) && (getTemperature(NTC1_PIN) >= Settings.Trigger1Temp)) || ((Settings.Trigger2Temp != 0) && (getTemperature(NTC2_PIN) >= Settings.Trigger2Temp)))
          {
            toStandbyMode();
          }
        }
    */

    // Turn Screen Saver on/off if it is activated and if no user input has been received during the defined number of seconds
  if (UIkey == KEY_NONE)
  {
    if ((!ScreenSaverIsOn && (millis() - mil_LastUserInput > (unsigned long)Settings.DisplayTimeout * 1000)) && Settings.ScreenSaverActive)
    {
      ScreenSaverOn();
    }
  }
  else if (appMode != APP_STANDBY_MODE)
    ScreenSaverOff();

    switch (UIkey)
    {
    case KEY_NONE:
      // If inactivity timer is set, go to standby if the set number of hours have passed since last user input
      if ((Settings.TriggerInactOffTimer > 0) && ((mil_LastUserInput + Settings.TriggerInactOffTimer * 3600000) < millis()))
        toStandbyMode();
      break;
    case KEY_BACK:
      break;
    case KEY_UP:
      // Turn volume up if we're not muted and we'll not exceed the maximum volume set for the currently selected input
      // TO DO: The checks for mute and MaxVol are done in setVolume so can be deleted here?
      if (!RuntimeSettings.Muted && (RuntimeSettings.CurrentVolume < Settings.Input[RuntimeSettings.CurrentInput].MaxVol))
        setVolume(RuntimeSettings.CurrentVolume + 1);
      break;
    case KEY_DOWN:
      // Turn volume down if we're not muted and we'll not get below the minimum volume set for the currently selected input
      // TO DO: The checks for mute and MinVol are done in setVolume so can be deleted here?
      if (!RuntimeSettings.Muted && (RuntimeSettings.CurrentVolume > Settings.Input[RuntimeSettings.CurrentInput].MinVol))
        setVolume(RuntimeSettings.CurrentVolume - 1);
      break;
    case KEY_LEFT:
    {
      setPrevInput();
      break;
    }
    case KEY_RIGHT:
    {
      setNextInput();
      break;
    }
    case KEY_1:
    case KEY_2:
    case KEY_3:
    case KEY_4:
    case KEY_5:
      setInput(UIkey - KEY_1);
      break;
    case KEY_PREVIOUS:
      // Switch to previous selected input (to allow for A-B comparison)
      setInput(RuntimeSettings.PrevSelectedInput);
      break;
    case KEY_MUTE:
      // toggle mute
      if (RuntimeSettings.Muted)
        unmute();
      else
        mute();
      right_display_update();
      break;
    case KEY_SELECT:
      // Set channel balance
      changeBalance();
      toAppNormalMode();
      break;
    case KEY_OFF:
      if (last_KEY_ONOFF + 5000 < millis()) // Cancel received KEY_ONOFF if it has been received within the last 5 seconds
      {
        last_KEY_ONOFF = millis();
        toStandbyMode();
      }
      break;
    }
    break;

  case APP_STANDBY_MODE:
  {
    // Do nothing if in APP_STANDBY_MODE - unless the user presses KEY_ONOFF. By the way: you don't need an IR remote: a doubleclick on encoder_2 is also KEY_ONOFF
    switch (UIkey)
    {
    case KEY_ON:
      if (last_KEY_ONOFF + 5000 < millis()) // Cancel received KEY_ONOFF if it has been received within the last 5 seconds
      {
        last_KEY_ONOFF = millis();
        startUp();
      }
      break;
    }
    /*- TO DO
    // Send temperature notification via websocket while in standby mode
    if (millis() > mil_onRefreshTemperatureDisplay + (TEMP_REFRESH_INTERVAL_STANDBY))
    {
      // notifyClients(getJSONTempValues());
      mil_onRefreshTemperatureDisplay = millis();
    }
    */
    break;
  }
  }
}


// Trigger 1 relay -> MCP23008 pin 2 Right
// Trigger 2 relay -> MCP23008 pin 1 Left


