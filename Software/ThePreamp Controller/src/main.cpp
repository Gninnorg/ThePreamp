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

#define ROTARY_ENCODER_STEPS 4

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
#define ROTARY1_CW_PIN 25
#define ROTARY1_CCW_PIN 26
#define ROTARY1_SW_PIN 34
#define ROTARY2_CW_PIN 27
#define ROTARY2_CCW_PIN 14
#define ROTARY2_SW_PIN 35

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

// Setup Rotary encoders ------------------------------------------------------
ClickEncoder *encoder1 = new ClickEncoder(ROTARY1_CW_PIN, ROTARY1_CCW_PIN, ROTARY1_SW_PIN, ROTARY_ENCODER_STEPS, LOW);
ClickEncoder::Button button1;
int16_t e1last, e1value;

ClickEncoder *encoder2 = new ClickEncoder(ROTARY2_CW_PIN, ROTARY2_CCW_PIN, ROTARY2_SW_PIN, ROTARY_ENCODER_STEPS, LOW);
ClickEncoder::Button button2;
int16_t e2last, e2value;

volatile int interruptCounter;
int totalInterruptCounter;

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
void IRAM_ATTR timerIsr()
{
  encoder1->service();
  encoder2->service();
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setupRotaryEncoders()
{
  pinMode(ROTARY1_CW_PIN, INPUT_PULLUP);
  pinMode(ROTARY1_CCW_PIN, INPUT_PULLUP);
  pinMode(ROTARY1_SW_PIN, INPUT); // No internal pullup resistor on this pin
  pinMode(ROTARY2_CW_PIN, INPUT_PULLUP);
  pinMode(ROTARY2_CCW_PIN, INPUT_PULLUP);
  pinMode(ROTARY2_SW_PIN, INPUT); // No internal pullup resistor on this pin
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &timerIsr, true);
  timerAlarmWrite(timer, 1000, true);
  timerAlarmEnable(timer);
}

// Setup Muses72323 -----------------------------------------------------------
Muses72323 muses(0, SPI_CS_MUSES_PIN); // Run at 500kHz

// Setup Relay Controller------------------------------------------------------
Adafruit_MCP23008 relayController;

// Function declarations
void toAppNormalMode();
void toStandbyMode();
void setup();
void setupWIFIsupport();
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
  
  setupWIFIsupport();

  // Set pin mode for control of power relay
  pinMode(POWER_CONTROL_PIN, OUTPUT);

  muses.begin();
  muses.setExternalClock(false);
  muses.setZeroCrossingOn(true);
    
  startUp();
}

void setupWIFIsupport()
{
  initSPIFFS();

  if (initWiFi())
  {
    // Web Server Root URL
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/index.html", "text/html"); });
    
    // Web : InputSelector
    server.on("/INPUT1", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/plain", String(setInput(0)));});

    server.on("/INPUT2", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/plain", String(setInput(1)));});

    server.on("/INPUT3", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/plain", String(setInput(2)));});

    server.on("/INPUT4", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/plain", String(setInput(3)));});

    server.on("/INPUT5", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/plain", String(setInput(4)));});

    server.on("/MUTE", HTTP_GET, [](AsyncWebServerRequest *request)
              { muteOutput(); request->send(200, "text/plain", "Mute");});

    server.on("/UNMUTE", HTTP_GET, [](AsyncWebServerRequest *request)
              { unmuteOutput(); request->send(200, "text/plain", "Unmute");});

    server.serveStatic("/", SPIFFS, "/");

    ElegantOTA.begin(&server);
    WebSerial.begin(&server); // WebSerial is accessible at "<IP Address>/webserial" in browser

    /* Attach Message Callback */
    WebSerial.onMessage([&](uint8_t *data, size_t len) {
    Serial.printf("Received %u bytes from WebSerial: ", len);
    Serial.write(data, len);
    debugln("");
    WebSerial.println("Received Data...");
    String input = "";
    String command = "";
    String value = "";
    for(size_t i=0; i < len; i++){
      input += char(data[i]);
    }
      int spaceIndex = input.indexOf(' ');
      if (spaceIndex > 0) {
        command = input.substring(0, spaceIndex);
        value = input.substring(spaceIndex + 1);
      } else {
        command = input;
        value = "";
      }
      command.trim();
      value.trim();
           
      WebSerial.print("Command: ");
      WebSerial.println(command);
      WebSerial.print("Value: ");
      WebSerial.println(value);

      if (command == "HELP") {
        WebSerial.println("IR_UP value");
        WebSerial.println("IR_DOWN value");
      }

      if (command == "EXPORT-SETTINGS") {
        String json = exportSettingsAsJson();
        WebSerial.println(json);
      }
    });

    server.begin();
  }
  else
  {
    // Setting up AP (Access Point) for WiFi configuration 
    debugln("Setting AP (Access Point)");

    WiFi.mode(WIFI_AP);
    WiFi.setTxPower(WIFI_POWER_19_5dBm); // Set maximum transmit power
    WiFi.softAP("ThePreAmp", NULL, 6, 0); // NULL sets an open Access Point
    dnsServer.start(53, "*", WiFi.softAPIP());

    IPAddress IP = WiFi.softAPIP();
    debug("AP IP address: ");
    debugln(IP);

    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
              { AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/style.css.gz", "text/css");
                response->addHeader("Content-Encoding", "gzip");
                request->send(response);
                debugln("style.css");
    });

    server.on("update.html", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(SPIFFS, "/update.html", "text/html");
      debug(request->url());
      debug(request->host());
      debug(": ");
      debugln("NotFound");
    });

    server.onNotFound([](AsyncWebServerRequest *request)
    {
      request->send(SPIFFS, "/wifi.html", "text/html");
      debug(request->url());
      debug(request->host());
      debug(": ");
      debugln("NotFound");
    });

    //What this statement ment for?
    //server.serveStatic("/", SPIFFS, "/");

    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request)
       {
      int params = request->params();
      for(size_t i=0;i<params;i++){
        const AsyncWebParameter* p = request->getParam(i);
        if(p->isPost()){
          // HTTP POST ssid value
          if (p->name() == PARAM_INPUT_1) {
            strcpy(Settings.ssid, p->value().c_str()); /* String copy*/
            debug("SSID set to: ");
            debugln(Settings.ssid);
          }
          // HTTP POST pass value
          if (p->name() == PARAM_INPUT_2) {
            strcpy(Settings.pass, p->value().c_str()); /* String copy */
            debug("Password set to: ");
            debugln(Settings.pass);
          }
          // HTTP POST ip value
          if (p->name() == PARAM_INPUT_3) {
            strcpy(Settings.ip, p->value().c_str()); /* String copy*/
            debug("IP Address set to: ");
            debugln(Settings.ip);
          }
          // HTTP POST gateway value
          if (p->name() == PARAM_INPUT_4) {
            strcpy(Settings.gateway, p->value().c_str()); /* String copy */
            debug("Gateway set to: ");
            debugln(Settings.gateway);
          }
          //Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
        }
      }

      writeSettingsToEEPROM();
      request->send(200, "text/plain", "Done. ESP will restart, connect to your router and go to IP address: " + String(Settings.ip));
      //oled.clear();
      //oled.setCursor(0, 1);
      //oled.print(F("Wifi is configured"));
      //oled.setCursor(0, 3);
      debugln("Restarting...");
      delay(3000);
      ESP.restart(); 
    });
    ElegantOTA.begin(&server);
    server.begin();

    // Display WiFi QR code
    left_display.clearBuffer();
    left_display.drawXBMP(0, 0, 64, 64, ThePreAmp_wifi_QR);
    left_display.setFont(u8g2_font_luBS18_tf);
    left_display.drawStr(74, 31, "Scan to");
    left_display.drawStr(74, 58, "setup WiFi");
    left_display.sendBuffer();

    right_display.clearBuffer();
    right_display.setFont(u8g2_font_luBS18_tf);
    right_display.drawStr(0, 31, "Push volume");
    right_display.drawStr(0, 58, "button to skip");
    right_display.sendBuffer();
    while (getUserCommand() != KEY_SELECT) {
       ElegantOTA.loop();
       dnsServer.processNextRequest();
    };
  }
}

void startUp()
{
  debugln("Starting up...");
  // Display logo
  left_display.clearBuffer();
  left_display.drawXBMP(77, 0, 130, 64, thePreAmpLogo);
  left_display.sendBuffer();

  right_display.clearBuffer();
  right_display.sendBuffer();
  delay(1000);

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


