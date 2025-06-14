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

#define ROTARY_ENCODER_STEPS 4

// To enable debug define DEBUG 1
// To disable debug define DEBUG 0
#define DEBUG 1
#if DEBUG == 1
#define debug(x) Serial.print(x); WebSerial.print(x);
#define debugln(x) Serial.println(x); WebSerial.println(x);
#else
#define debug(x)
#define debugln(x)
#endif

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
const long interval = 10000; // interval to wait for Wi-Fi connection (milliseconds)

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

const uint8_t thePreAmpLogo[] = {

// 'ThePreAmp 130x64 BW', 130x64px
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x3f, 0xf0, 0x3f, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x01, 0x00, 0xfe, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0xf0, 0x03, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x0f, 0x00, 0x00, 0xc0, 0x07, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x03, 0x00, 0x00, 0x00, 
0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 
0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 
0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 
0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 
0x00, 0x00, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x0f, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x9f, 0x00, 0x00, 0x00, 0xf8, 0x07, 0x00, 0x00, 0x00, 
0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x9f, 0x01, 0x00, 0x00, 0xfc, 0x1f, 0x00, 0x00, 
0x00, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x80, 0x01, 0x00, 0x00, 0x0c, 0x30, 0x00, 
0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x80, 0x01, 0x00, 0x00, 0x0c, 0x60, 
0x00, 0x00, 0x00, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x80, 0x01, 0x00, 0x00, 0x0c, 
0x40, 0x00, 0x00, 0x00, 0x30, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x80, 0x7d, 0x80, 0x0f, 
0x0c, 0x40, 0xf2, 0xe0, 0x07, 0x30, 0x02, 0xf2, 0xf0, 0x41, 0x7c, 0x00, 0x40, 0x80, 0xff, 0xc0, 
0x3f, 0x0c, 0x40, 0xfe, 0xf1, 0x0f, 0x10, 0x06, 0xfe, 0xf9, 0xc3, 0xff, 0x00, 0x40, 0x80, 0x83, 
0x61, 0x60, 0x0c, 0x60, 0x0e, 0x18, 0x18, 0x18, 0x06, 0x06, 0x0f, 0xc6, 0x81, 0x01, 0x40, 0x80, 
0x01, 0x31, 0x40, 0x0c, 0x60, 0x06, 0x0c, 0x10, 0x08, 0x0c, 0x06, 0x06, 0xc4, 0x01, 0x03, 0x40, 
0x80, 0x01, 0x33, 0xc0, 0x0c, 0x30, 0x06, 0x0c, 0x30, 0x0c, 0x0c, 0x02, 0x06, 0xcc, 0x00, 0x03, 
0x40, 0x80, 0x01, 0xf3, 0xff, 0xfc, 0x1f, 0x06, 0xfc, 0x3f, 0x0c, 0x18, 0x02, 0x06, 0xcc, 0x00, 
0x02, 0x40, 0x80, 0x01, 0xf3, 0xff, 0xfc, 0x07, 0x06, 0xfc, 0x3f, 0xfe, 0x1f, 0x02, 0x06, 0xcc, 
0x00, 0x02, 0x40, 0x80, 0x01, 0x13, 0x00, 0x0c, 0x00, 0x06, 0x0c, 0x00, 0xfe, 0x1f, 0x02, 0x06, 
0xcc, 0x00, 0x02, 0x40, 0x80, 0x01, 0x33, 0x00, 0x0c, 0x00, 0x06, 0x0c, 0x00, 0x03, 0x30, 0x02, 
0x06, 0xcc, 0x00, 0x03, 0x40, 0x80, 0x01, 0x33, 0x20, 0x0c, 0x00, 0x06, 0x18, 0x10, 0x03, 0x30, 
0x02, 0x06, 0xcc, 0x01, 0x03, 0x40, 0x80, 0x01, 0xe3, 0x70, 0x0c, 0x00, 0x06, 0x38, 0x9c, 0x01, 
0x60, 0x02, 0x06, 0xcc, 0xc3, 0x01, 0x40, 0x80, 0x01, 0xc3, 0x1f, 0x0c, 0x00, 0x06, 0xf0, 0x87, 
0x01, 0x60, 0x02, 0x06, 0xc4, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0x80, 0x07, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x80, 0x03, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0xc0, 
0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 
0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 
0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 
0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x03, 
0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 
0x0f, 0x00, 0x00, 0xc0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x3f, 0x00, 0x00, 0xf0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0xfc, 0x01, 0x00, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0xf0, 0x3f, 0xf0, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// 'ThePreAmp wifi QR', 64x64px
const uint8_t ThePreAmp_wifi_QR [] = {
	0x00, 0x80, 0x0f, 0xfc, 0xc1, 0xc3, 0x01, 0x00, 0x00, 0x80, 0x0f, 0xfc, 0xc1, 0xc3, 0x01, 0x00, 
	0xfc, 0x9f, 0xff, 0xff, 0x3f, 0xf3, 0xf9, 0x3f, 0xfc, 0x9f, 0xff, 0xff, 0x3f, 0xf3, 0xf9, 0x3f, 
	0x0c, 0x98, 0x03, 0x7f, 0xc6, 0xc3, 0x19, 0x30, 0x0c, 0x98, 0x03, 0x7f, 0xc6, 0xc3, 0x19, 0x30, 
	0x0c, 0x98, 0x03, 0x7f, 0xc6, 0xc3, 0x19, 0x30, 0x0c, 0x98, 0x03, 0xe3, 0xc7, 0xfc, 0x19, 0x30, 
	0x0c, 0x98, 0x03, 0xe3, 0xc7, 0xfc, 0x19, 0x30, 0x0c, 0x98, 0xf3, 0x00, 0xf8, 0xc0, 0x19, 0x30, 
	0x0c, 0x98, 0xf3, 0x00, 0xf8, 0xc0, 0x19, 0x30, 0xfc, 0x9f, 0x03, 0x00, 0x00, 0xf3, 0xf9, 0x3f, 
	0xfc, 0x9f, 0x03, 0x00, 0x00, 0xf3, 0xf9, 0x3f, 0x00, 0x80, 0x33, 0x63, 0xc6, 0xcc, 0x01, 0x00, 
	0x00, 0x80, 0x33, 0x63, 0xc6, 0xcc, 0x01, 0x00, 0x00, 0x80, 0x03, 0x00, 0xc0, 0xc0, 0x01, 0x00, 
	0xff, 0xff, 0xc3, 0x80, 0xf9, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xc3, 0x80, 0xf9, 0xc0, 0xff, 0xff, 
	0x0c, 0x80, 0x3f, 0x63, 0xf8, 0xfc, 0x01, 0xf0, 0x0c, 0x80, 0x3f, 0x63, 0xf8, 0xfc, 0x01, 0xf0, 
	0xf3, 0xe1, 0x33, 0xff, 0xc1, 0x33, 0x1e, 0xfe, 0xf3, 0xe1, 0x33, 0xff, 0xc1, 0x33, 0x1e, 0xfe, 
	0xfc, 0x07, 0xf0, 0xff, 0x3f, 0xc3, 0x87, 0x0f, 0xfc, 0x07, 0xf0, 0xff, 0x3f, 0xc3, 0x87, 0x0f, 
	0xf3, 0xe7, 0x03, 0x7f, 0xc6, 0xf3, 0x01, 0xce, 0xf3, 0xe7, 0x03, 0x7f, 0xc6, 0xf3, 0x01, 0xce, 
	0xf3, 0xe7, 0x03, 0x7f, 0xc6, 0xf3, 0x01, 0xce, 0x80, 0x01, 0x0c, 0xe0, 0x07, 0x3c, 0x86, 0xf1, 
	0x80, 0x01, 0x0c, 0xe0, 0x07, 0x3c, 0x86, 0xf1, 0x7f, 0xfe, 0x0f, 0x03, 0xf8, 0x0f, 0x00, 0x3e, 
	0x7f, 0xfe, 0x0f, 0x03, 0xf8, 0x0f, 0x00, 0x3e, 0xff, 0x1f, 0xc0, 0x1f, 0x00, 0xcf, 0xff, 0xff, 
	0xff, 0x1f, 0xc0, 0x1f, 0x00, 0xcf, 0xff, 0xff, 0x80, 0x79, 0x3c, 0x83, 0xf9, 0x0f, 0x80, 0xf1, 
	0x80, 0x79, 0x3c, 0x83, 0xf9, 0x0f, 0x80, 0xf1, 0x80, 0x79, 0x3c, 0x83, 0xf9, 0x0f, 0x80, 0xf1, 
	0x70, 0x80, 0xc3, 0x60, 0x38, 0xcc, 0x1f, 0xfe, 0x70, 0x80, 0xc3, 0x60, 0x38, 0xcc, 0x1f, 0xfe, 
	0x7c, 0x7e, 0x00, 0xe3, 0xc1, 0xf3, 0xff, 0x3f, 0x7c, 0x7e, 0x00, 0xe3, 0xc1, 0xf3, 0xff, 0x3f, 
	0x8c, 0x19, 0xcc, 0xfc, 0x3f, 0xc0, 0x7f, 0x00, 0x8c, 0x19, 0xcc, 0xfc, 0x3f, 0xc0, 0x7f, 0x00, 
	0x7c, 0x78, 0x3c, 0x63, 0x06, 0x33, 0x86, 0xf1, 0x7c, 0x78, 0x3c, 0x63, 0x06, 0x33, 0x86, 0xf1, 
	0x7c, 0x78, 0x3c, 0x63, 0x06, 0x33, 0x86, 0xf1, 0x8c, 0x19, 0x3c, 0xff, 0x07, 0x0c, 0x80, 0xc1, 
	0x8c, 0x19, 0x3c, 0xff, 0x07, 0x0c, 0x80, 0xc1, 0xff, 0xff, 0xc3, 0x1c, 0xf8, 0xcf, 0x1f, 0xce, 
	0xff, 0xff, 0xc3, 0x1c, 0xf8, 0xcf, 0x1f, 0xce, 0x00, 0x80, 0xff, 0x1f, 0x00, 0xcc, 0x19, 0xfe, 
	0x00, 0x80, 0xff, 0x1f, 0x00, 0xcc, 0x19, 0xfe, 0xfc, 0x9f, 0xf3, 0x9f, 0x39, 0xcf, 0x9f, 0x0f, 
	0xfc, 0x9f, 0xf3, 0x9f, 0x39, 0xcf, 0x9f, 0x0f, 0x0c, 0x98, 0xc3, 0x63, 0x38, 0x0c, 0x00, 0x00, 
	0x0c, 0x98, 0xc3, 0x63, 0x38, 0x0c, 0x00, 0x00, 0x0c, 0x98, 0xc3, 0x63, 0x38, 0x0c, 0x00, 0x00, 
	0x0c, 0x98, 0xf3, 0xe0, 0xc1, 0x03, 0x9e, 0xff, 0x0c, 0x98, 0xf3, 0xe0, 0xc1, 0x03, 0x9e, 0xff, 
	0x0c, 0x98, 0xf3, 0x80, 0xff, 0xf0, 0x07, 0xc0, 0x0c, 0x98, 0xf3, 0x80, 0xff, 0xf0, 0x07, 0xc0, 
	0xfc, 0x9f, 0x3f, 0xff, 0xc7, 0xf0, 0x87, 0x31, 0xfc, 0x9f, 0x3f, 0xff, 0xc7, 0xf0, 0x87, 0x31, 
	0x00, 0x80, 0xf3, 0x7f, 0xfe, 0x00, 0x80, 0xf1, 0x00, 0x80, 0xf3, 0x7f, 0xfe, 0x00, 0x80, 0xf1
};

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
#define POWER_CONTROL_PIN 2
#define INPUT_HT_PASSTHROUGH 0
#define INPUT_NORMAL 1
#define INPUT_INACTIVATED 2

unsigned long mil_On = millis(); // Holds the millis from last power on (or restart)
bool ScreenSaverIsOn = false; // Used to indicate whether the screen saver is running or not
unsigned long mil_LastUserInput = millis(); // Used to keep track of the time of the last user interaction (part of the screen saver timing)
unsigned long mil_onRefreshTemperatureDisplay; // Used to time how often the display of temperatures is updated

// Update intervals for the display/notification of temperatures
#define TEMP_REFRESH_INTERVAL 10000         // Interval while on
#define TEMP_REFRESH_INTERVAL_STANDBY 60000 // Interval while in standby

//  What state is active?
enum AppModeValues
{
  APP_NORMAL_MODE,
  APP_BALANCE_MODE,
  APP_STANDBY_MODE
};

byte appMode = APP_NORMAL_MODE;

// Enumerated set of possible inputs from the user
enum UserInput
{
  KEY_NONE,    // No input
  KEY_UP,      // Rotary 1 turned CW or IR
  KEY_DOWN,    // Rotary 1 turned CCW or IR
  KEY_REPEAT,  // IR
  KEY_SELECT,  // Rotary 1 switch pressed or IR
  KEY_RIGHT,   // Rotary 2 turned CW or IR
  KEY_LEFT,    // Rotary 2 turned CCW or IR
  KEY_BACK,    // Rotary 2 switch pressed or IR
  KEY_1,       // IR
  KEY_2,       // IR
  KEY_3,       // IR
  KEY_4,       // IR
  KEY_5,       // IR
  KEY_MUTE,    // IR
  KEY_ON,      // IR
  KEY_OFF,     // IR
  KEY_PREVIOUS // IR
};

byte UIkey; // holds the last received user input (from rotary encoders or IR)
byte lastReceivedInput = KEY_NONE;
unsigned long last_KEY_ONOFF = millis(); // Used to ensure that fast repetition of KEY_ONOFF is not accepted

struct InputSettings
{
  byte Active;
  char Name[8];
  byte MaxVol;  // The maximum volume allowed for this input in steps
  byte MinVol;  // The minimum volume allowed for this input in steps
  byte Gain;

};

// This holds all the settings of the controller
// It is saved to the I2C EEPROM on the first run and read back into memory on subsequent runs
// The settings can be changed from the menu and the user can also chose to reset to default values if something goes wrong
// On startup of the controller it is checked if the EEPROM contains valid data by checking if the Version field equals the VERSION defined by the source code. If they are not the same default values will be written to EEPROM
// This is created as a union to be able to serialize/deserialize the data when writing and reading to/from the EEPROM
typedef union
{
  struct
  {
    char ssid[33];    // Wifi network SSDI
    char pass[33];    // Wifi network password
    char ip[16];      // Wifi network assigned IP address
    char gateway[16]; // Wifi network gateway IP address

    byte VolumeSteps;    // The number of steps of the volume control
    byte MinAttenuation; // Minimum attenuation in -dB (as 0 db equals no attenuation this is equal to the highest volume allowed)
    byte MaxAttenuation; // Maximum attenuation in -dB (as -111.5 db is the limit of the Muses72323 this is equal to the lowest volume possible). We only keep this setting as a positive number, and we do also only allow the user to set the value in 1 dB steps
    byte MaxStartVolume; // If StoreSetLevel is true, then limit the volume to the specified value when the controller is powered on
    byte MuteLevel;      // The level to be set when Mute is activated by the user. The Mute function of the Muses72323 is activated if 0 is specified
    byte RecallSetLevel; // Remember/store the volume level for each separate input

    uint64_t IR_ON;               // IR data to be interpreted as ON
    uint64_t IR_OFF;              // IR data to be interpreted as OFF
    uint64_t IR_UP;               // IR data to be interpreted as UP
    uint64_t IR_DOWN;             // IR data to be interpreted as DOWN
    uint64_t IR_REPEAT;           // IR data to be interpreted as REPEAT (ie Apple remotes sends a specific code, if a key is held down to indicate repeat of the previously sent code
    uint64_t IR_LEFT;             // IR data to be interpreted as LEFT
    uint64_t IR_RIGHT;            // IR data to be interpreted as RIGHT
    uint64_t IR_SELECT;           // IR data to be interpreted as SELECT
    uint64_t IR_BACK;             // IR data to be interpreted as BACK
    uint64_t IR_MUTE;             // IR data to be interpreted as MUTE
    uint64_t IR_PREVIOUS;         // IR data to be interpreted as "switch to previous selected input"
    uint64_t IR_1;                // IR data to be interpreted as 1 (to select input 1 directly)
    uint64_t IR_2;                // IR data to be interpreted as 2
    uint64_t IR_3;                // IR data to be interpreted as 3
    uint64_t IR_4;                // IR data to be interpreted as 5
    uint64_t IR_5;                // IR data to be interpreted as 4
    
    struct InputSettings Input[5]; // Settings for all 5 inputs
    bool ExtPowerRelayTrigger;     // Enable triggering of relay for external power (we use it to control the power of the Mezmerize)
    byte Trigger1Active;           // 0 = the trigger is not active, 1 = the trigger is active
    byte Trigger1Type;             // 0 = momentary, 1 = latching
    byte Trigger1OnDelay;          // Seconds from controller power up to activation of trigger. The default delay allows time for the output relay of the Mezmerize to be activated before we turn on the power amps. The selection of an input of the Mezmerize will also be delayed.
    byte Trigger1Temp;             // Temperature protection: if the temperature is measured to the set number of degrees Celcius (via the LDRs), the controller will attempt to trigger a shutdown of the connected power amps (if set to 0, the temperature protection is not active
    byte Trigger2Active;           // 0 = the trigger is not active, 1 = the trigger is active
    byte Trigger2Type;             // 0 = momentary, 1 = latching
    byte Trigger2OnDelay;          // Seconds from controller power up to activation of trigger. The default delay allows time for the output relay of the Mezmerize to be activated before we turn on the power amps. The selection of an input of the Mezmerize will also be delayed.
    byte Trigger2Temp;             // Temperature protection: if the temperature is measured to the set number of degrees Celcius (via the LDRs), the controller will attempt to trigger a shutdown of the connected power amps (if set to 0, the temperature protection is not active)
    byte TriggerInactOffTimer;     // Hours without user interaction before automatic power down (0 = never)
    byte ScreenSaverActive;        // 0 = the display will stay on/not be dimmed, 1 = the display will be dimmed to the specified level after a specified period of time with no user input
    byte DisplayOnLevel;           // The contrast level of the display when it is on, 0 = 25%, 1 = 50%, 2 = 75%, 3 = 100%
    byte DisplayDimLevel;          // The contrast level of the display when screen saver is active. 0 = off, 1 = 3, 2 = 7 ... 32 = 127. If DisplayDimLevel = 0 the display will be turned off when the screen saver is active (to reduce electrical noise)
    byte DisplayTimeout;           // Number of seconds before the screen saver is activated.
    byte DisplayVolume;            // 0 = no display of volume, 1 = show step number, 2 = show as -dB
    byte DisplaySelectedInput;     // 0 = the name of the active input is not shown on the display (ie. if only one input is used), 1 = the name of the selected input is shown on the display
    byte DisplayTemperature1;      // 0 = do not display the temperature measured by NTC 1, 1 = display in number of degrees Celcious, 2 = display as graphical representation, 3 = display both
    byte DisplayTemperature2;      // 0 = do not display the temperature measured by NTC 2, 1 = display in number of degrees Celcious, 2 = display as graphical representation, 3 = display both
    float Version;                 // Used to check if data read from the EEPROM is valid with the compiled version of the code - if not a reset to default settings is necessary and they must be written to the EEPROM
  };
  byte data[318]; // Allows us to be able to write/read settings from EEPROM byte-by-byte (to avoid specific serialization/deserialization code)
} mySettings;

mySettings Settings; // Holds all the current settings
void setSettingsToDefault(void);

typedef union
{
  struct
  {
    byte CurrentInput;      // The number of the currently set input
    byte CurrentVolume;     // The currently set volume step 
    bool Muted;             // Indicates if we are in mute mode or not
    byte InputLastVol[5];   // The last set volume for each input
    byte InputLastBal[5];   // The last set balance for each input: 127 = no balance shift (values < 127 = shift balance to the left channel, values > 127 = shift balance to the right channel)
    byte PrevSelectedInput; // Holds the input selected before the current one (enables switching back and forth between two inputs, eg. while A-B testing)
    float Version;          // Used to check if data read from the EEPROM is valid with the compiled version of the compiled code - if not a reset to defaults is necessary and they must be written to the EEPROM
  };
  byte data[18]; // Allows us to be able to write/read settings from EEPROM byte-by-byte (to avoid specific serialization/deserialization code)
} myRuntimeSettings;

myRuntimeSettings RuntimeSettings;

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

// Setup EEPROM ---------------------------------------------------------------
#define EEPROM_Address 0x50
extEEPROM eeprom(kbits_64, 1, 32); // Set to use 24C64 Eeprom - look in the datasheet for capacity in kbits (kbits_64) and page size in bytes (32) if you use another type 

// Function declarations
void setup();
void initSPIFFS();
bool initWiFi();
void setupWIFIsupport();
void startUp();
void loop();
void writeSettingsToEEPROM();
void readSettingsFromEEPROM();
void writeDefaultSettingsToEEPROM();
void writeRuntimeSettingsToEEPROM();
void readRuntimeSettingsFromEEPROM();
void readUserSettingsFromEEPROM();
void writeUserSettingsToEEPROM();
void setSettingsToDefault();
void setVolume(int16_t);
void left_display_update();
void right_display_update();
void drawSignalStrength(int);
float getTemperature(uint8_t pinNmbr);
void drawTemperatureMeasurements(void);
byte getUserInput();
void toAppNormalMode();
void toStandbyMode();
void ScreenSaverOn();
void ScreenSaverOff();
boolean setInput(uint8_t);
void setPrevInput();
void setNextInput();
void mute();
void unmute();
bool changeBalance();
void displayBalance(byte);
int calculateAttenuation(byte logicalStep, byte maxLogicalSteps, byte minAttenuation_dB, byte maxAttenuation_dB);
void setTrigger1On();
void setTrigger1Off();
void setTrigger2On();
void setTrigger2Off();
void setOutputRelayOff();
void setOutputRelayOn();
String exportSettingsAsJson();

void setup() {
  // Serial port for debugging purposes
  #if DEBUG == 1
    Serial.begin(115200);
  #endif
  
  SPI.begin();
  Wire.begin();

  right_display.setBusClock(2000000);
  right_display.begin();
  right_display.setPowerSave(0);
  right_display.setFont(u8g2_font_inb63_mn); 
  
  left_display.setBusClock(2000000);
  left_display.begin();
  left_display.setPowerSave(0);
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
    
  startUp();
}

// Initialize SPIFFS
void initSPIFFS()
{
  if (!SPIFFS.begin(true))
  {
    debugln("An error has occurred while mounting SPIFFS");
  }
  debugln("SPIFFS mounted successfully");
}

// Initialize WiFi
bool initWiFi()
{ 
  if (Settings.ssid == "" || Settings.ip == "")
  {
    debugln("Undefined SSID or IP address.");
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // Set maximum transmit power
  localIP.fromString(Settings.ip);
  localGateway.fromString(Settings.gateway);

  if (!WiFi.config(localIP, localGateway, subnet))
  {
    debugln("STA Failed to configure");
    return false;
  }

  WiFi.begin(Settings.ssid, Settings.pass);
  
  debug("Connecting to WiFi... "); debugln(Settings.ssid);

  unsigned long currentMillis = millis();
  previousMillis = currentMillis;

  while (WiFi.status() != WL_CONNECTED)
  {
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
      debugln("Failed to connect.");
      return false;
    }
  }
  debug("Connected to WiFi. IP: "); debugln(WiFi.localIP());
  return true;
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
        

    server.serveStatic("/", SPIFFS, "/");

    ElegantOTA.begin(&server);
    WebSerial.begin(&server); // WebSerial is accessible at "<IP Address>/webserial" in browser

    /* Attach Message Callback */
    WebSerial.onMessage([&](uint8_t *data, size_t len) {
    Serial.printf("Received %u bytes from WebSerial: ", len);
    Serial.write(data, len);
    Serial.println();
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
    while (getUserInput() != KEY_SELECT) {
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
  delay(2000);

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
  setOutputRelayOn();

  left_display_update();
  right_display_update();

  UIkey = KEY_NONE;
  lastReceivedInput = KEY_NONE;
}

void loop()
{
  ElegantOTA.loop();
  WebSerial.loop();
  
  UIkey = getUserInput();

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


// Function definitions

// Write Settings to EEPROM
void writeSettingsToEEPROM()
{
  // Write the settings to the EEPROM
  eeprom.begin(extEEPROM::twiClock400kHz);
  eeprom.write(0, Settings.data, sizeof(Settings));
}

// Read Settings from EEPROM
void readSettingsFromEEPROM()
{
  // Read settings from EEPROM
  eeprom.begin(extEEPROM::twiClock400kHz);
  eeprom.read(0, Settings.data, sizeof(Settings));
}

// Write Default Settings and RuntimeSettings to EEPROM - called if the EEPROM data is not valid or if the user chooses to reset all settings to default value
void writeDefaultSettingsToEEPROM()
{
  // Read default settings into Settings
  setSettingsToDefault();
  // Write the settings to the EEPROM
  writeSettingsToEEPROM();
  // Write the runtime settings to the EEPROM
  writeRuntimeSettingsToEEPROM();
}

// Write the current runtime settings to EEPROM - called if a power drop is detected or if the EEPROM data is not valid or if the user chooses to reset all settings to default values
void writeRuntimeSettingsToEEPROM()
{
  // Write the settings to the EEPROM
  eeprom.begin(extEEPROM::twiClock400kHz);
  eeprom.write(sizeof(Settings) + 1, RuntimeSettings.data, sizeof(RuntimeSettings));
}

// Read the last runtime settings from EEPROM
void readRuntimeSettingsFromEEPROM()
{
  // Read the settings from the EEPROM
  eeprom.begin(extEEPROM::twiClock400kHz);
  eeprom.read(sizeof(Settings) + 1, RuntimeSettings.data, sizeof(RuntimeSettings));
}

// Read the user defined settings from EEPROM
void readUserSettingsFromEEPROM()
{
  // Read the settings from the EEPROM
  eeprom.begin(extEEPROM::twiClock400kHz);
  eeprom.read(sizeof(Settings) + sizeof(RuntimeSettings) + 1, Settings.data, sizeof(Settings));
}

// Read the user defined settings from EEPROM
void writeUserSettingsToEEPROM()
{
  // Write the user settings to the EEPROM
  eeprom.begin(extEEPROM::twiClock400kHz);
  eeprom.write(sizeof(Settings) + sizeof(RuntimeSettings) + 1, Settings.data, sizeof(Settings));
}

// Loads default settings into Settings and RuntimeSettings - this is only done when the EEPROM does not contain valid settings or when reset is chosen by user in the menu
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

/*
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
*/   
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
  RuntimeSettings.InputLastBal[0] = 127; // 127 = no balance shift (values < 127 = shift balance to the left channel, values > 127 = shift balance to the right channel)
  RuntimeSettings.InputLastBal[1] = 127;
  RuntimeSettings.InputLastBal[2] = 127;
  RuntimeSettings.InputLastBal[3] = 127;
  RuntimeSettings.InputLastBal[4] = 127;
  RuntimeSettings.PrevSelectedInput = 0;
  RuntimeSettings.Version = VERSION;
}

void setVolume(int16_t newVolumeStep)
{
  if (appMode == APP_NORMAL_MODE || appMode == APP_BALANCE_MODE)
  {
    if (newVolumeStep < Settings.Input[RuntimeSettings.CurrentInput].MinVol)
      newVolumeStep = Settings.Input[RuntimeSettings.CurrentInput].MinVol;
    else if (newVolumeStep > Settings.Input[RuntimeSettings.CurrentInput].MaxVol)
      newVolumeStep = Settings.Input[RuntimeSettings.CurrentInput].MaxVol;

    if (!RuntimeSettings.Muted)
    {
      if (Settings.Input[RuntimeSettings.CurrentInput].Active != INPUT_HT_PASSTHROUGH)
        RuntimeSettings.CurrentVolume = newVolumeStep;
      else
        RuntimeSettings.CurrentVolume = Settings.Input[RuntimeSettings.CurrentInput].MaxVol; // Set to max volume
      RuntimeSettings.InputLastVol[RuntimeSettings.CurrentInput] = RuntimeSettings.CurrentVolume;

      int Attenuation = calculateAttenuation(RuntimeSettings.CurrentVolume, Settings.VolumeSteps, Settings.MinAttenuation, Settings.MaxAttenuation);
      muses.setVolume(Attenuation, Attenuation);
      if (RuntimeSettings.InputLastBal[RuntimeSettings.CurrentInput] == 127 || RuntimeSettings.InputLastBal[RuntimeSettings.CurrentInput] < 118 || RuntimeSettings.InputLastBal[RuntimeSettings.CurrentInput] > 136) // Both channels same attenuation
        muses.setVolume(Attenuation, Attenuation);
      else if (RuntimeSettings.InputLastBal[RuntimeSettings.CurrentInput] < 127) // Shift balance to the left channel by lowering the right channel - TO DO: seems like the channels is reversed in the Muses library??
        muses.setVolume(Attenuation + (127 - RuntimeSettings.InputLastBal[RuntimeSettings.CurrentInput]), Attenuation);
      else if (RuntimeSettings.InputLastBal[RuntimeSettings.CurrentInput] > 127) // Shift balance to the right channel by lowering the left channel - TO DO: seems like the channels is reversed in the Muses library??
        muses.setVolume(Attenuation, Attenuation + (RuntimeSettings.InputLastBal[RuntimeSettings.CurrentInput] - 127));
    }
    if (appMode == APP_NORMAL_MODE)
      right_display_update();
  }
}

void left_display_update(void)
{
  // Display the name of the current input (but only if it has been chosen to be so by the user)
  if (Settings.DisplaySelectedInput)
  {
    if (ScreenSaverIsOn)
      ScreenSaverOff();
    
    left_display.setFont(u8g2_font_inb42_mr);
      
    // Calculate the width of the text 
    int16_t textWidth = left_display.getStrWidth(Settings.Input[RuntimeSettings.CurrentInput].Name);
    // Calculate the height of the text 
    int16_t textHeight = left_display.getMaxCharHeight(); 
    // Calculate the x-position to center the text horizontally 
    int16_t xPos = (256 - textWidth) / 2; 
    // Calculate the y-position to center the text vertically 
    int16_t yPos = 52; 
    // Draw the text 
    left_display.clearBuffer();
    left_display.drawStr(xPos, yPos, Settings.Input[RuntimeSettings.CurrentInput].Name);  
    left_display.sendBuffer(); 
  }
}

void right_display_update(void)
{
  right_display.clearBuffer();

  // Display the volume or mute status
  if (Settings.DisplayVolume)
  {
    right_display.setFont(u8g2_font_inb63_mn);
    if (!RuntimeSettings.Muted)
    {
      // If show volume in steps
      if (Settings.DisplayVolume == 1)
      {
        // Display volume as step
        // Convert the integer to a string 
        char buffer[10]; 
        // Buffer to hold the string representation of the integer 
        snprintf(buffer, sizeof(buffer), "%d", RuntimeSettings.CurrentVolume);
        // Calculate the width of the text 
        int16_t textWidth = right_display.getStrWidth(buffer); 
        // Calculate the x-position to center the text horizontally 
        int16_t xPos = (256 - textWidth) / 2; 
        // Calculate the y-position to center the text vertically 
        int16_t yPos = 63; 
        // Draw the text 
        right_display.drawStr(xPos, yPos, buffer);
      }
      else 
      {
        // Display volume as -dB - RuntimeSettings.CurrentAttennuation are converted to -dB and multiplied by 10 to be able to show 0.25 dB steps        
        char buffer[10]; 
        // Buffer to hold the string representation of the integer 
        snprintf(buffer, sizeof(buffer), "%d", (calculateAttenuation(RuntimeSettings.CurrentVolume, Settings.VolumeSteps, Settings.MinAttenuation, Settings.MaxAttenuation) / 4) );
        // Calculate the width of the text 
        int16_t textWidth = right_display.getStrWidth(buffer); 
        // Calculate the height of the text 
        int16_t textHeight = right_display.getMaxCharHeight(); 
        // Calculate the x-position to center the text horizontally 
        int16_t xPos = (256 - textWidth) / 2; 
        // Calculate the y-position to center the text vertically 
        int16_t yPos = 63; 
        // Draw the text 
        right_display.drawStr(xPos,yPos, buffer);
      }
    }
    else
    {
      right_display.setFont(u8g2_font_inb63_mn);
      // Calculate the width of the text 
      int16_t textWidth = right_display.getStrWidth("MUTE"); 
      // Calculate the height of the text 
      int16_t textHeight = right_display.getMaxCharHeight(); 
      // Calculate the x-position to center the text horizontally 
      int16_t xPos = (256 - textWidth) / 2; 
      // Calculate the y-position to center the text vertically 
      int16_t yPos = 63; 
      // Draw the text 
      right_display.drawStr(xPos,yPos, "MUTE");  
    }
  }

  // Display the WiFi status
  switch(WiFi.status())
  {
    case WL_CONNECTED:
      drawSignalStrength(WiFi.RSSI());
      break;
  }

  // Display temperature measurements?
  if (Settings.DisplayTemperature1 || Settings.DisplayTemperature2)
  {
    drawTemperatureMeasurements();
  }

  right_display.sendBuffer();
  if (ScreenSaverIsOn)
      ScreenSaverOff();
}

void drawSignalStrength(int rssi) 
{ 
  if (rssi >= -55) {
    right_display.drawBox(232,4,4,4);
    right_display.drawBox(237,3,4,5);
    right_display.drawBox(242,2,4,6);
    right_display.drawBox(247,1,4,7);
    right_display.drawBox(252,0,4,8);
  } else if (rssi >= -67) {
    right_display.drawBox(232,4,4,4);
    right_display.drawBox(237,3,4,5);
    right_display.drawBox(242,2,4,6);
    right_display.drawBox(247,1,4,7);
    right_display.drawFrame(252,0,4,8);
  } else if (rssi >= -70) {
    right_display.drawBox(232,4,4,4);
    right_display.drawBox(237,3,4,5);
    right_display.drawBox(242,2,4,6);
    right_display.drawFrame(247,1,4,7);
    right_display.drawFrame(252,0,4,8);
  } else if (rssi >= -80) {
    right_display.drawBox(232,4,4,4);
    right_display.drawBox(237,3,4,5);
    right_display.drawFrame(242,2,4,6);
    right_display.drawFrame(247,1,4,7);
    right_display.drawFrame(252,0,4,8);
  } else if (rssi >= -90) {
     right_display.drawBox(232,4,4,4);
     right_display.drawFrame(237,3,4,5);
     right_display.drawFrame(242,2,4,6);
     right_display.drawFrame(247,1,4,7);
     right_display.drawFrame(252,0,4,8);
  } else {
     right_display.drawFrame(232,4,4,4);
     right_display.drawFrame(237,3,4,5);
     right_display.drawFrame(242,2,4,6);
     right_display.drawFrame(247,1,4,7);
     right_display.drawFrame(252,0,4,8);
  } 
}

// Read temperature from NTC on specified pin on ADS1115
float getTemperature(uint8_t pinNmbr)
{
  // 0 = A0 = Right channel NTC, 1 = A1 = Left channel NTC

  float Vin = 3.3;   // Input voltage 3.3V for ESP32
  float Vout = 0;    // Measured voltage
  float Rref = 10000; // Reference resistor's value in ohms
  float Rntc = 0;    // Measured resistance of NTC+
  float Temp;

  // Read the analog value from the specified pin
  //int16_t adcValue = ads1115.readADC_SingleEnded(pinNmbr);
  int16_t adcValue = 15000; // TO DO: Implement reading of ADC value from ADS1115
  Vout = (adcValue * Vin) / 32767.0; // Convert ADC value to voltage

  Rntc = Rref * (Vin / Vout - 1); // Calculate the resistance of the NTC

  if (Rntc < 0)
    Temp = 0;
  else
    Temp = (-25.37 * log(Rntc)) + 239.43; // Calculate temperature from resistance

  if (Temp < 0)
    Temp = 0;
  else if (Temp > 65)
    Temp = 65;

  /*
    debug("Voltage: ");
  debugln(Vout);
  debug(" Resistance: ");
  debugln(Rntc);
  debug(" Temperature: ");
  debugln(Temp);
  */
  
  return Temp;
} 


void drawTemperatureMeasurements(void)
{
  // TO DO: Implement handling of Settings.DisplayTemperature1 and Settings.DisplayTemperature2 
  
  right_display.drawFrame(232,34,24,14);
  right_display.drawFrame(232,50,24,14);
  
  right_display.setFontMode(1);
  right_display.setDrawColor(1);

  right_display.drawFrame(232,34,24,14);
  int tempRight = static_cast<int>(getTemperature(0)); // Get temperature for right channel and convert to integer  
  right_display.drawBox(234,36,map(tempRight, 0, 65, 0, 20),10);

  right_display.drawFrame(232,50,24,14);
  int tempLeft = static_cast<int>(getTemperature(1)); // Get temperature for left channel and convert to integer
  right_display.drawBox(234,52,map(tempLeft, 0, 65, 0, 20),10);

  right_display.setDrawColor(2);
  right_display.setFont(u8g2_font_profont10_mf);

  char tempRightStr[3];
  snprintf(tempRightStr, sizeof(tempRightStr), "%d", tempRight);
  right_display.drawStr(239, 44, tempRightStr);

  char tempLeftStr[3];
  snprintf(tempLeftStr, sizeof(tempLeftStr), "%d", tempLeft);
  right_display.drawStr(239, 60, tempLeftStr);
}

// Returns input from the user - enumerated to be the same value no matter if input is from encoders or IR remote
byte getUserInput()
{

  if (interruptCounter > 0)
  {
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);

    totalInterruptCounter++;
  }

  byte receivedInput = KEY_NONE;

  // Read input from encoder 1
  e1value += encoder1->getValue();

  if (e1value != e1last)
  {
    if (e1value > e1last)
      receivedInput = KEY_UP;
    if (e1value < e1last)
      receivedInput = KEY_DOWN;
    e1last = e1value;
  }

  // Check if button on encoder 1 is clicked
  button1 = encoder1->getButton();
  switch (button1)
  {
  case ClickEncoder::Clicked:
    receivedInput = KEY_SELECT;
    break;
  default:
    break;
  }

  // Read input from encoder 2
  e2value += encoder2->getValue();

  if (e2value != e2last)
  {
    if (e2value > e2last)
      receivedInput = KEY_RIGHT;
    if (e2value < e2last)
      receivedInput = KEY_LEFT;
    e2last = e2value;
  }

  // Check if button on encoder 2 is clicked
  button2 = encoder2->getButton();
  switch (button2)
  {
  case ClickEncoder::Clicked:
    receivedInput = KEY_BACK;
    break;
  
  case ClickEncoder::DoubleClicked:
    if (appMode == APP_STANDBY_MODE)
      receivedInput = KEY_ON;
    else 
      receivedInput = KEY_OFF;
    break;
  
  default:
    break;
  }

  // Check if any input from the IR remote
  if (irrecv.decode(&IRresults))
  {
      debug("IR code: "); debug(uint64ToString(IRresults.value, HEX).c_str());debugln("");
      // Map the received IR input to UserInput values
      if (IRresults.value == Settings.IR_UP)
        receivedInput = KEY_UP;
      else if (IRresults.value == Settings.IR_DOWN)
        receivedInput = KEY_DOWN;
      else if (IRresults.value == Settings.IR_LEFT)
        receivedInput = KEY_LEFT;
      else if (IRresults.value == Settings.IR_RIGHT)
        receivedInput = KEY_RIGHT;
      else if (IRresults.value == Settings.IR_SELECT)
        receivedInput = KEY_SELECT;
      else if (IRresults.value == Settings.IR_BACK)
        receivedInput = KEY_BACK;
      else if (IRresults.value == Settings.IR_MUTE)
        receivedInput = KEY_MUTE;
      else if (appMode == APP_STANDBY_MODE && IRresults.value == Settings.IR_ON)
        receivedInput = KEY_ON;
      else if (appMode == APP_NORMAL_MODE && IRresults.value == Settings.IR_OFF)
        receivedInput = KEY_OFF;
      else if (IRresults.value == Settings.IR_1)
        receivedInput = KEY_1;
      else if (IRresults.value == Settings.IR_2)
        receivedInput = KEY_2;
      else if (IRresults.value == Settings.IR_3)
        receivedInput = KEY_3;
      else if (IRresults.value == Settings.IR_4)
        receivedInput = KEY_4;
      else if (IRresults.value == Settings.IR_5)
        receivedInput = KEY_5;
      else if (IRresults.value == Settings.IR_PREVIOUS)
        receivedInput = KEY_PREVIOUS;
      else if (IRresults.value == Settings.IR_REPEAT)
      {
        receivedInput = KEY_REPEAT;
        if (lastReceivedInput == KEY_UP)
          receivedInput = KEY_UP;
        else if (lastReceivedInput == KEY_DOWN)
          receivedInput = KEY_DOWN;
      }
    lastReceivedInput = receivedInput;
    irrecv.resume();  // Receive the next value
  }
  if (receivedInput != KEY_NONE) { 
    mil_LastUserInput = millis();
    debug("getUserInput: "); debugln(receivedInput); 
  }
  return (receivedInput);
}

void toAppNormalMode()
{
  debugln("toAppNormalMode");
  left_display_update();
  right_display_update();
  appMode = APP_NORMAL_MODE;
}

void toStandbyMode()
{
  debugln("toStandbyMode");
  appMode = APP_STANDBY_MODE;
  writeRuntimeSettingsToEEPROM();
  mute();
  setOutputRelayOff();
  left_display.clearDisplay();
  right_display.clearDisplay();
  setTrigger1Off();
  setTrigger2Off();
  if (Settings.ExtPowerRelayTrigger)
  {
    digitalWrite(POWER_CONTROL_PIN, LOW);
  }
  last_KEY_ONOFF = millis();
}

void ScreenSaverOn(void)
{
  debugln("Screensaver on");
  ScreenSaverIsOn = true;
  left_display.clearDisplay();
  right_display.clearDisplay();
}

void ScreenSaverOff(void)
{
  if (ScreenSaverIsOn) 
  {
    debugln("Screensaver off");
    ScreenSaverIsOn = false;
    left_display_update();
    right_display_update();
  }
}

boolean setInput(uint8_t NewInput)
{
  boolean result = false;
  if (Settings.Input[NewInput].Active != INPUT_INACTIVATED && NewInput >= 0 && NewInput <= 4 && appMode == APP_NORMAL_MODE)
  {
    if (!RuntimeSettings.Muted)
      mute();

    // Input 1 relay -> RuntimeSettings.CurrentInput = 0 -> MCP23008 pin 7
    // Input 2 relay -> RuntimeSettings.CurrentInput = 1 -> MCP23008 pin 6
    // Input 3 relay -> RuntimeSettings.CurrentInput = 2 -> MCP23008 pin 5
    // Input 4 relay -> RuntimeSettings.CurrentInput = 3 -> MCP23008 pin 4
    // Input 5 relay -> RuntimeSettings.CurrentInput = 4 -> MCP23008 pin 3

    // Unselect currently selected input
    relayController.digitalWrite(7 - RuntimeSettings.CurrentInput, LOW);

    // Save the currently selected input to enable switching between two inputs
    RuntimeSettings.PrevSelectedInput = RuntimeSettings.CurrentInput;

    // Select new input
    muses.setGain(Settings.Input[RuntimeSettings.CurrentInput].Gain);
    RuntimeSettings.CurrentInput = NewInput;


    if (Settings.RecallSetLevel)
      RuntimeSettings.CurrentVolume = RuntimeSettings.InputLastVol[RuntimeSettings.CurrentInput];
    else if (RuntimeSettings.CurrentVolume > Settings.Input[RuntimeSettings.CurrentInput].MaxVol)
      RuntimeSettings.CurrentVolume = Settings.Input[RuntimeSettings.CurrentInput].MaxVol;
    else if (RuntimeSettings.CurrentVolume < Settings.Input[RuntimeSettings.CurrentInput].MinVol)
      RuntimeSettings.CurrentVolume = Settings.Input[RuntimeSettings.CurrentInput].MinVol;
        
    relayController.digitalWrite(7 - NewInput, HIGH);
    
    if (RuntimeSettings.Muted)
      unmute();
    else
      setVolume(RuntimeSettings.CurrentVolume);

    left_display_update();
    result = true;
  }
  return result;
}

// Select the next active input (DOWN)
void setPrevInput(void)
{
  debug("setPrevInput: "); debugln(RuntimeSettings.CurrentInput); 
  byte nextInput = (RuntimeSettings.CurrentInput == 0) ? 4 : RuntimeSettings.CurrentInput - 1;

  debug("setPrevInput: try "); debugln(nextInput);
  while (Settings.Input[nextInput].Active == INPUT_INACTIVATED)
  {
    nextInput = (nextInput == 0) ? 5 : nextInput - 1;
    debug("setPrevInput: try again "); debugln(nextInput);
  }
  debug("setPrevInput: found "); debugln(nextInput);
  setInput(nextInput);
}

// Select the next active input (UP)
void setNextInput(void)
{
  debug("setNextInput: "); debugln(RuntimeSettings.CurrentInput); 
  byte nextInput = (RuntimeSettings.CurrentInput == 4) ? 0 : RuntimeSettings.CurrentInput + 1;
  
  debug("setNextInput: try "); debugln(nextInput);
  while (Settings.Input[nextInput].Active == INPUT_INACTIVATED)
  {
    nextInput = (nextInput == 4) ? 0 : nextInput + 1;
    debug("setNextInput: try again "); debugln(nextInput);
  }
  debug("setNextInput: found "); debugln(nextInput);
  setInput(nextInput);
}

void mute()
{
  if (Settings.MuteLevel)
  // TO DO: This does not consider if channel balance has been set - it might not be a problem at all
    muses.setVolume(calculateAttenuation(Settings.MuteLevel, Settings.VolumeSteps, Settings.MinAttenuation, Settings.MaxAttenuation), calculateAttenuation(Settings.MuteLevel, Settings.VolumeSteps, Settings.MinAttenuation, Settings.MaxAttenuation));
  else
    muses.mute();
  RuntimeSettings.Muted = true;
}

void unmute()
{
  RuntimeSettings.Muted = false;
  setVolume(RuntimeSettings.CurrentVolume);
}

// Change channel balance for current input
// The balance between channels can be shifted up to 4.5 dB in 0.5 dB steps
// 127 = no balance shift (values < 127 = shift balance to the left channel, values > 127 = shift balance to the right channel)
bool changeBalance()
{
  bool complete = false;
  bool result = false;
  byte OldValue = RuntimeSettings.InputLastBal[RuntimeSettings.CurrentInput];
  byte NewValue = RuntimeSettings.InputLastBal[RuntimeSettings.CurrentInput];

  appMode = APP_BALANCE_MODE;

  // Display the screen
  /*- TO DO
  oled.clear();
  oled.print("Balance");
  displayBalance(NewValue);

  while (!complete)
  {
    mil_LastUserInput = millis(); // Prevent the screen saver to kick in while editing
    switch (getUserInput())
    {
    case KEY_UP:
      if (NewValue < 136)
      {
        NewValue++;
        RuntimeSettings.InputLastBal[RuntimeSettings.CurrentInput] = NewValue;
        setVolume(RuntimeSettings.CurrentVolume);
        displayBalance(NewValue);
      }
      break;
    case KEY_DOWN:
      if (NewValue > 118)
      {
        NewValue--;
        RuntimeSettings.InputLastBal[RuntimeSettings.CurrentInput] = NewValue;
        setVolume(RuntimeSettings.CurrentVolume);
        displayBalance(NewValue);
      }
      break;
    case KEY_SELECT:
      if (NewValue != OldValue)
      {
        writeRuntimeSettingsToEEPROM();
        result = true;
      }
      complete = true;
      break;
    case KEY_BACK:
      // Exit without saving new value
      RuntimeSettings.InputLastBal[RuntimeSettings.CurrentInput] = OldValue;
      // TO DO Set volume
      result = false;
      complete = true;
      break;
    default:
      break;
    }
  }
  */
  return result;
}

void displayBalance(byte Value)
{
  /*- TO DO
  oled.setCursor(1, 1);
  oled.print("---------=---------");
  oled.setCursor(Value - 117, 1);
  oled.write(31);
  oled.setCursor(1, 2);
  if (Value < 127)
    oled.printf("L         %.1f dB R", (127 - Value) * -0.5);
  else if (Value == 127)
    oled.print("L                 R");
  else if (Value > 127)
    oled.printf("L %.1f dB         R", (Value - 127) * -0.5);
  */
}

int calculateAttenuation(byte logicalStep, byte maxLogicalSteps, byte minAttenuation_dB, byte maxAttenuation_dB) 
{
  /*
  ** Return the attenuation required by the setvolume function of the Muses72323 based upon the selected step, configured 
  ** number of steps and the configured minimum and maximum attenuation in dBs.
  ** 
  ** Parameters
  **   logicalStep:       Selected step in potentiometer ladder from which you want the attenuation calculated
  **                        logicalStep = 1                (equals max_dB attenuation)
  **                        logicalStep = maxLogicalsteps  (equals min_db attenuation)
  **   maxLogicalSteps:   Number of desired step for you potentiometer. 
  **                        Maximum number of step = (max_dB-min_dB) * 2
  **   minAttenuation_dB: Minimum attenuation for the potentiometer         0 dB = absolute minimum 
  **   maxAttenuation_dB: Maximum attenuation for the potentionmeter   111.75 dB = absolute maximum
  **
  ** Constraints
  **   maxAttenuation_dB < minAttenuation_dB
  **   logicalStep <= maxLogicalSteps
  **   maxLogicalSteps >= 10 
  **   maxLogicalSteps <= (maxAttenuation_dB - minAttenuation_dB) / 4
  **
  ** If the above constraints are not meet the calculateAttenuation() will return 0 (mute);
  **
  */
  debug("Logical step: "); debugln(logicalStep);

  if (minAttenuation_dB >= maxAttenuation_dB ||
      logicalStep < 1 ||
      logicalStep > maxLogicalSteps ||
      maxLogicalSteps < 10 ||
      maxLogicalSteps <= ((maxAttenuation_dB - minAttenuation_dB) / 4)) return minAttenuation_dB*-4;

    // Calculate the total attenuation range
    float attenuationRange = maxAttenuation_dB - minAttenuation_dB;
    // Calculate the attenuation per logical step
    float attenuationPerStep = attenuationRange / (maxLogicalSteps - 1);
    // Calculate the attenuation in dB for the given logical step (reversed)
    float attenuation = maxAttenuation_dB - (logicalStep - 1) * attenuationPerStep;
    // Round the attenuation to the nearest 0.25 dB
    attenuation = round(attenuation * 4) / 4;
    // Calculate the volume step
    int volumeStep = static_cast<int>(attenuation * -4);
    debug("Attenuation: "); debugln((int)attenuation);
    debug("Volume step: "); debugln(volumeStep);
    return volumeStep;
    
}

// Trigger 1 relay -> MCP23008 pin 2 Right
// Trigger 2 relay -> MCP23008 pin 1 Left

void setTrigger1On()
{
  if (Settings.Trigger1Active)
  {
    relayController.digitalWrite(2, HIGH);
    if (Settings.Trigger1Type == 0) // Momentary
    {
      delay(200);
      relayController.digitalWrite(2, LOW);
    }
  }
  
}

void setTrigger1Off()
{
  if (Settings.Trigger1Active)
  {
    if (Settings.Trigger1Type == 0) // Momentary
    {
      relayController.digitalWrite(2, HIGH);
      delay(200);
    }
    relayController.digitalWrite(2, LOW);
  }
}

void setTrigger2On()
{
  if (Settings.Trigger2Active)
  {
    relayController.digitalWrite(1, HIGH);
    if (Settings.Trigger2Type == 0) // Momentary
    {
      delay(200);
      relayController.digitalWrite(1, LOW);
    }
  }
}

void setTrigger2Off()
{
  if (Settings.Trigger2Active)
  {
    if (Settings.Trigger2Type == 0) // Momentary
    {
      relayController.digitalWrite(1, HIGH);
      delay(200);
    }
    relayController.digitalWrite(1, LOW);
  }
}

// Mute relay -> MCP23008 pin 0

void setOutputRelayOff()
{
  relayController.digitalWrite(0, LOW);
}

void setOutputRelayOn()
{
  relayController.digitalWrite(0, HIGH);
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