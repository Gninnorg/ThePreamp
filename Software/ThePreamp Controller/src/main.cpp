/*
**
**    Controller for ThePreAmp
**
**    Copyright (c) 2024 Carsten Grønning, Jan Abkjer Tofft
**
*/

#define VERSION (float)0.98

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP23008.h>
#include <extEEPROM.h>
#include <ClickEncoder.h>
#define ROTARY_ENCODER_STEPS 4
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <Muses72320.h>

// To enable debug define DEBUG 1
// To disable debug define DEBUG 0
#define DEBUG 1
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

#undef minimum
#ifndef minimum
#define minimum(a, b) ((a) < (b) ? (a) : (b))
#endif

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

//  What state are active?
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
  KEY_ONOFF,   // IR
  KEY_PREVIOUS // IR
};

byte UIkey; // holds the last received user input (from rotary encoders or IR)
byte lastReceivedInput = KEY_NONE;
unsigned long last_KEY_ONOFF = millis(); // Used to ensure that fast repetition of KEY_ONOFF is not accepted

struct InputSettings
{
  byte Active;
  char Name[11];
  byte MaxVol;
  byte MinVol;
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
    byte MaxAttenuation; // Maximum attenuation in -dB (as -111.5 db is the limit of the Muses72320 this is equal to the lowest volume possible). We only keep this setting as a positive number, and we do also only allow the user to set the value in 1 dB steps
    byte MaxStartVolume; // If StoreSetLevel is true, then limit the volume to the specified value when the controller is powered on
    byte MuteLevel;      // The level to be set when Mute is activated by the user. The Mute function of the Muses72320 is activated if 0 is specified
    byte RecallSetLevel; // Remember/store the volume level for each separate input

    float ADC_Calibration; // Used for calibration of the ADC readings when reading temperatures from the attached NTCs. The value differs (quite a lot) between ESP32's

    uint64_t IR_ONOFF;            // IR data to be interpreted as ON/OFF - switch between running and suspend mode (and turn triggers off)
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
    uint64_t IR_5;                // IR data to be interpreted as 5
    uint64_t IR_4;                // IR data to be interpreted as 4
    uint64_t IR_6;                // IR data to be interpreted as 6
    
    struct InputSettings Input[6]; // Settings for all 6 inputs
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
  byte data[232]; // Allows us to be able to write/read settings from EEPROM byte-by-byte (to avoid specific serialization/deserialization code)
} mySettings;

mySettings Settings; // Holds all the current settings
void setSettingsToDefault(void);

typedef union
{
  struct
  {
    byte CurrentInput;      // The number of the currently set input
    byte CurrentVolume;     // The currently set volume
    bool Muted;             // Indicates if we are in mute mode or not
    byte InputLastVol[6];   // The last set volume for each input
    byte InputLastBal[6];   // The last set balance for each input: 127 = no balance shift (values < 127 = shift balance to the left channel, values > 127 = shift balance to the right channel)
    byte PrevSelectedInput; // Holds the input selected before the current one (enables switching back and forth between two inputs, eg. while A-B testing)
    float Version;          // Used to check if data read from the EEPROM is valid with the compiled version of the compiled code - if not a reset to defaults is necessary and they must be written to the EEPROM
  };
  byte data[20]; // Allows us to be able to write/read settings from EEPROM byte-by-byte (to avoid specific serialization/deserialization code)
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
  pinMode(ROTARY2_CW_PIN, INPUT_PULLUP);
  pinMode(ROTARY2_CCW_PIN, INPUT_PULLUP);
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &timerIsr, true);
  timerAlarmWrite(timer, 1000, true);
  timerAlarmEnable(timer);
}

// Setup Muses72323 -----------------------------------------------------------
// TO DO
Muses72320 muses(0);

// Setup Relay Controller------------------------------------------------------
Adafruit_MCP23008 relayController;

// Setup EEPROM ---------------------------------------------------------------
#define EEPROM_Address 0x50
extEEPROM eeprom(kbits_64, 1, 32); // Set to use 24C64 Eeprom - look in the datasheet for capacity in kbits (kbits_64) and page size in bytes (32) if you use another type 

// Function declarations
void startUp(void);
void toStandbyMode(void);
void toAppNormalMode(void);
void ScreenSaverOff(void);
void ScreenSaverOn(void);
void setTrigger1On(void);
void setTrigger2On(void);
void setTrigger1Off(void);
void setTrigger2Off(void);
void displayTemperatures(void);
void displayTempDetails(float, uint8_t, uint8_t, uint8_t);
float readVoltage(byte);
float getTemperature(uint8_t);
void left_display_update(void);
void right_display_update(void);
int16_t getAttenuation(uint8_t, uint8_t, uint8_t, uint8_t);
void setVolume(int16_t);
bool changeBalance(void);
void displayBalance(byte);
void mute(void);
void unmute(void);
boolean setInput(uint8_t);
void setPrevInput(void);
void setNextInput(void);
void readSettingsFromEEPROM(void);
void writeSettingsToEEPROM(void);
void writeDefaultSettingsToEEPROM(void);
void readRuntimeSettingsFromEEPROM(void);
void writeRuntimeSettingsToEEPROM(void);
void readUserSettingsFromEEPROM(void);
void writeUserSettingsToEEPROM(void);
void setVolume(int16_t);
byte getUserInput(void);

void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);
  
  SPI.begin();
  Wire.begin();

  right_display.setBusClock(2000000);
  right_display.begin();
  right_display.setPowerSave(0);
  right_display.setFont(u8g2_font_cu12_tr); // u8g2_font_inr38_mf
  
  left_display.setBusClock(2000000);
  left_display.begin();
  left_display.setPowerSave(0);
  left_display.setFont(u8g2_font_cu12_tr); 
  
  setupRotaryEncoders();
  
  relayController.begin();
  // Define all pins as OUTPUT and disable all relays
  for (byte pin = 0; pin <= 7; pin++)
  {
    relayController.pinMode(pin, OUTPUT);
    relayController.digitalWrite(pin, LOW);
  }
  
  muses.begin();

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
    right_display.setBusClock(2000000);
    right_display.clearBuffer();
    right_display.drawStr(0, 63, "Reset");  // 0 left, 0 top bottom appx 100
    right_display.sendBuffer();
    delay(2000);
    writeDefaultSettingsToEEPROM();
  }
  
  // Connect to Wifi
  // TO DO
  //setupWIFIsupport();

  // Set pin mode for control of power relay
  pinMode(POWER_CONTROL_PIN, OUTPUT);

  startUp();
}

void startUp()
{
  // Display logo
  left_display.setBusClock(2000000);
  left_display.clearBuffer();
  left_display.drawXBMP(77, 0, 130, 64, thePreAmpLogo);
  left_display.sendBuffer();
  delay(2000);

  /*oled.lcdOn();
  oled.clear();

  // Turn on Mezmerize B1 Buffer via power on/off relay
  if (Settings.ExtPowerRelayTrigger)
  {
    digitalWrite(POWER_RELAY_PIN, HIGH);
  }
  */ 
  // The controller is now ready - save the timestamp
  mil_On = millis();

  /*
  // If triggers are active then wait for the set number of seconds and turn them on
  unsigned long delayTrigger1 = (Settings.Trigger1Active) ? (mil_On + Settings.Trigger1OnDelay * 1000) : 0;
  unsigned long delayTrigger2 = (Settings.Trigger2Active) ? (mil_On + Settings.Trigger2OnDelay * 1000) : 0;

  if (delayTrigger1 || delayTrigger2)
  {
    oled.clear();
    oled.print(F("Wait..."));
  }

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
      if (Settings.Trigger1Active && delayTrigger1 != 0)
        oled.print3x3Number(2, 1, (delayTrigger1 - millis()) / 1000, false);
    }

    if (millis() > delayTrigger2 && delayTrigger2 != 0)
    {
      setTrigger2On();
      delayTrigger2 = 0;
      // oled.print3x3Number(11, 1, 0, false);
    }
    else
    {
      if (Settings.Trigger2Active && delayTrigger2 != 0)
        oled.print3x3Number(11, 1, (delayTrigger2 - millis()) / 1000, false);
    }
  }
  oled.clear();
*/
  ScreenSaverOff();
  appMode = APP_NORMAL_MODE;
  // Keep start volume for current input lower than max allowed start volume
  RuntimeSettings.InputLastVol[RuntimeSettings.CurrentInput] = minimum(RuntimeSettings.InputLastVol[RuntimeSettings.CurrentInput], Settings.MaxStartVolume); // Avoid setting volume higher than MaxStartVol
  setInput(RuntimeSettings.CurrentInput);

  left_display_update();
  right_display_update();

  UIkey = KEY_NONE;
  lastReceivedInput = KEY_NONE;
}

void loop()
{
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
    case KEY_ONOFF:
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
    case KEY_ONOFF:
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
  Settings.ADC_Calibration = 1.0; // TO DO Remove
  Settings.VolumeSteps = 60;
  Settings.MinAttenuation = 0;
  Settings.MaxAttenuation = 60;
  Settings.MaxStartVolume = Settings.VolumeSteps;
  Settings.MuteLevel = 0;
  Settings.RecallSetLevel = true;
  Settings.IR_UP = 0x1;
  Settings.IR_DOWN = 0x2;
  Settings.IR_REPEAT = 0x3;
  Settings.IR_LEFT = 0x4;
  Settings.IR_RIGHT = 0x5;
  Settings.IR_SELECT = 0x6;
  Settings.IR_BACK = 0x7;
  Settings.IR_MUTE = 0x8;
  Settings.IR_PREVIOUS = 0x9;
  Settings.IR_ONOFF = 0xA;
  Settings.IR_1 = 0xB;
  Settings.IR_2 = 0xC;
  Settings.IR_3 = 0xD;
  Settings.IR_4 = 0xE;
  Settings.IR_5 = 0xF;
  Settings.IR_6 = 0x11;
  Settings.Input[0].Active = INPUT_NORMAL;
  strcpy(Settings.Input[0].Name, "Input 1   ");
  Settings.Input[0].MaxVol = Settings.VolumeSteps;
  Settings.Input[0].MinVol = 0;
  Settings.Input[1].Active = INPUT_NORMAL;
  strcpy(Settings.Input[1].Name, "Input 2   ");
  Settings.Input[1].MaxVol = Settings.VolumeSteps;
  Settings.Input[1].MinVol = 0;
  Settings.Input[2].Active = INPUT_NORMAL;
  strcpy(Settings.Input[2].Name, "Input 3   ");
  Settings.Input[2].MaxVol = Settings.VolumeSteps;
  Settings.Input[2].MinVol = 0;
  Settings.Input[3].Active = INPUT_NORMAL;
  strcpy(Settings.Input[3].Name, "Input 4   ");
  Settings.Input[3].MaxVol = Settings.VolumeSteps;
  Settings.Input[3].MinVol = 0;
  Settings.Input[4].Active = INPUT_NORMAL;
  strcpy(Settings.Input[4].Name, "Input 5   ");
  Settings.Input[4].MaxVol = Settings.VolumeSteps;
  Settings.Input[4].MinVol = 0;
  Settings.Input[5].Active = INPUT_NORMAL;
  strcpy(Settings.Input[5].Name, "Input 6   ");
  Settings.Input[5].MaxVol = Settings.VolumeSteps;
  Settings.Input[5].MinVol = 0;
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
  RuntimeSettings.InputLastVol[5] = 0;
  RuntimeSettings.InputLastBal[0] = 127; // 127 = no balance shift (values < 127 = shift balance to the left channel, values > 127 = shift balance to the right channel)
  RuntimeSettings.InputLastBal[1] = 127;
  RuntimeSettings.InputLastBal[2] = 127;
  RuntimeSettings.InputLastBal[3] = 127;
  RuntimeSettings.InputLastBal[4] = 127;
  RuntimeSettings.InputLastBal[5] = 127;
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

      int Attenuation = getAttenuation(Settings.VolumeSteps, RuntimeSettings.CurrentVolume, Settings.MinAttenuation, Settings.MaxAttenuation);
      
      muses.setVolume(Attenuation);
      if (RuntimeSettings.InputLastBal[RuntimeSettings.CurrentInput] == 127 || RuntimeSettings.InputLastBal[RuntimeSettings.CurrentInput] < 118 || RuntimeSettings.InputLastBal[RuntimeSettings.CurrentInput] > 136) // Both channels same attenuation
        muses.setVolume(Attenuation);
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
    
    left_display.setBusClock(2000000);
    left_display.clearBuffer();
    left_display.setCursor(0, 63);
    left_display.print(Settings.Input[RuntimeSettings.CurrentInput].Name);  
    left_display.sendBuffer();  
  }
}

void right_display_update(void)
{
  if (Settings.DisplayVolume)
  {
    if (ScreenSaverIsOn)
      ScreenSaverOff();

    if (!RuntimeSettings.Muted)
    {
      right_display.setBusClock(2000000);
      // If show volume in steps
      if (Settings.DisplayVolume == 1)
      {
        right_display.clearBuffer();
        right_display.setCursor(0, 63);
        right_display.print(RuntimeSettings.CurrentVolume);  // Display volume as step
        right_display.sendBuffer();        
      }
      else // Show volume in -dB (-99.5 to 0)
      {
        right_display.clearBuffer();
        right_display.setCursor(0, 63);
        right_display.print((getAttenuation(Settings.VolumeSteps, RuntimeSettings.CurrentVolume, Settings.MinAttenuation, Settings.MaxAttenuation) / 2) * -10);  // Display volume as -dB - RuntimeSettings.CurrentAttennuation are converted to -dB and multiplied by 10 to be able to show 0.5 dB steps
        right_display.sendBuffer();
      }
    }
    else
    {
      right_display.setBusClock(2000000);
      right_display.clearBuffer();
      right_display.setCursor(0, 63);
      right_display.print("MUTE");  
      right_display.sendBuffer();
    }
  }
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
    receivedInput = KEY_ONOFF;
    break;
  
  default:
    break;
  }

  // Check if any input from the IR remote
  if (irrecv.decode(&IRresults))
  {
      debug("IR code: "); debugln(uint64ToString(IRresults.value, HEX).c_str());
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
      else if (IRresults.value == Settings.IR_ONOFF)
        receivedInput = KEY_ONOFF;
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
  left_display.clearDisplay();
  right_display.clearDisplay();
  /*- TO DO
  oled.setCursor(0, 1);
  oled.print(F("Going to sleep!"));
  oled.setCursor(11, 3);
  oled.print(F("...zzzZZZ"));
  setTrigger1Off();
  setTrigger2Off();
  if (Settings.ExtPowerRelayTrigger)
  {
    digitalWrite(POWER_RELAY_PIN, LOW);
  }
  last_KEY_ONOFF = millis();
  delay(3000);
  oled.lcdOff();
  */
  last_KEY_ONOFF = millis();
}

void ScreenSaverOn(void)
{
  /*- TO DO - clean up settings etc to not support dimlevel
  if (Settings.DisplayDimLevel == 0)
      oled.lcdOff();
   else
      oled.backlight(Settings.DisplayDimLevel * 4 - 1);
  */
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

    // Unselect currently selected input
    relayController.digitalWrite(RuntimeSettings.CurrentInput, LOW);

    // Save the currently selected input to enable switching between two inputs
    RuntimeSettings.PrevSelectedInput = RuntimeSettings.CurrentInput;

    // Select new input
    RuntimeSettings.CurrentInput = NewInput;
    relayController.digitalWrite(NewInput, HIGH);

    if (Settings.RecallSetLevel)
      RuntimeSettings.CurrentVolume = RuntimeSettings.InputLastVol[RuntimeSettings.CurrentInput];
    else if (RuntimeSettings.CurrentVolume > Settings.Input[RuntimeSettings.CurrentInput].MaxVol)
      RuntimeSettings.CurrentVolume = Settings.Input[RuntimeSettings.CurrentInput].MaxVol;
    else if (RuntimeSettings.CurrentVolume < Settings.Input[RuntimeSettings.CurrentInput].MinVol)
      RuntimeSettings.CurrentVolume = Settings.Input[RuntimeSettings.CurrentInput].MinVol;
    setVolume(RuntimeSettings.CurrentVolume);
    if (RuntimeSettings.Muted)
      unmute();

    left_display_update();
    result = true;
  }
  return result;
}

// Select the next active input (DOWN)
void setPrevInput(void)
{
  byte nextInput = (RuntimeSettings.CurrentInput == 0) ? 4 : RuntimeSettings.CurrentInput - 1;

  while (Settings.Input[nextInput].Active == INPUT_INACTIVATED)
  {
    nextInput = (nextInput == 0) ? 4 : nextInput - 1;
  }
  setInput(nextInput);
}

// Select the next active input (UP)
void setNextInput(void)
{
  byte nextInput = (RuntimeSettings.CurrentInput == 4) ? 0 : RuntimeSettings.CurrentInput + 1;
  while (Settings.Input[nextInput].Active == INPUT_INACTIVATED)
  {
    nextInput = (nextInput > 4) ? 0 : nextInput + 1;
  }
  setInput(nextInput);
}

void mute()
{
  if (Settings.MuteLevel)
    muses.setVolume(getAttenuation(Settings.VolumeSteps, Settings.MuteLevel, Settings.MinAttenuation, Settings.MaxAttenuation));
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

int16_t getAttenuation(uint8_t steps, uint8_t selStep, uint8_t min_dB, uint8_t max_dB)
{
  /*
  ** Return the attenuation required by the setvolume function of the Muses72320 based upon the configured 
  ** number of steps, the selected step and the configured minimum and maximum attenuation in dBs.
  **
  ** The purpose of the algoritm is the divide the potentionmeter into to sections. To get a more even step
  ** size in terms of the experienced sound preassure. One section will have larger attenuation step sizes.
  ** This will be in section starting from the maximum attenuation. Another section will have smaller step sizes.
  ** This will be in section closer to the minimum attenuation.
  ** 
  ** Parameters
  **   steps   : Number of desired step for you potentiometer. 
  **             Maximum number of step = (max_dB-min_dB) * 2 
  **   selStep : Selected step in potentiometer ladder from which you wan't the attenuation calculated
  **             selStep = 0      (equals max_dB attenuation)
  **             selStep = steps  (equals min_db attenuation)
  **   min_dB  : Minimum attenuation for the potentiometer      0 dB = absolute minimum 
  **   max_dB  : Maximum attenuation for the potentionmeter   111 dB = absolute maximum
  **
  ** Constraints
  **   max_dB < min_dB
  **   selStep <= steps
  **   steps >= 10 
  **   steps <= (max_dB - min_dB) / 2
  **
  ** If the above constraints are not meet the getAttenuation() will return 223 (111.5 max attenuation);
  **
  */
  debug("steps: "); debug(steps); debug(" selectedStep: "); debug(selStep); debug(" min_dB: "); debug(min_dB); debug(" max_dB: "); debugln(max_dB);
  if (min_dB >= max_dB ||
      selStep > steps ||
      steps < 10 ||
      steps <= ((max_dB - min_dB) / 2)) return -223;

  // Calculate attenuation range in dB
  uint8_t att_dB = max_dB - min_dB;

  // Calculate step size in DB for attenuation steps
  float sizeOfMajorSteps = round(pow(2.0, att_dB / steps) - 0.5);
  float sizeOfMinorSteps = sizeOfMajorSteps / 2;

  // Calculate number of minor steps for section with minor steps 
  // Use as many steps as possible for minor steps
  uint8_t numberOfMinorSteps = (sizeOfMajorSteps * steps - att_dB) / sizeOfMinorSteps;

  // return calculated for total attenuation for selected step off_set equals min_db
  return minimum((min_dB + 
                  // total attenuation in dB from number of selected steps in minor step section
                  // Start using minor steps - we want as many of these as possible.
                  // Equals when attenuation is close to 0 the steps should be as fine as possible
                  minimum(steps - selStep, numberOfMinorSteps) * sizeOfMinorSteps +   
                  // total attenuation in dB from number of selected step in major step section
                  // When every minor step is used start using large steps
                  // Equals when attenuation is close to max remaing steps should be the major ones.
                  max(steps - numberOfMinorSteps - selStep, 0) * sizeOfMajorSteps),
                  // total attenuation cannot exceed max_db   
                  max_dB) * 
                  // Total attenuation in db * 2 to calculation value in 1/2 dB steps 
                  -2;                                           
}
