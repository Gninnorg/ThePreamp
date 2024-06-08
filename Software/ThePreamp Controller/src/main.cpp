/*
**
**    Controller for ThePreAmp
**
**    Copyright (c) 2024 Carsten Grønning, Jan Abkjer Tofft
**
*/

#define VERSION (float)0.99

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
#define ROTARY2_CW_PIN 27
#define ROTARY2_CCW_PIN 14
#define ROTARY2_SW_PIN 35
#define ROTARY1_CW_PIN 25
#define ROTARY1_CCW_PIN 26
#define ROTARY1_SW_PIN 34
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
extEEPROM eeprom(kbits_64, 1, 32); // Set to use 24C64 Eeprom - if you use another type look in the datasheet for capacity in kbits (kbits_64) and page size in bytes (32)

// Function declarations
void startUp(void);
void toStandbyMode(void);
void toAppNormalMode(void);
void ScreenSaverOff(void);
void setTrigger1On(void);
void setTrigger2On(void);
void setTrigger1Off(void);
void setTrigger2Off(void);
void displayTemperatures(void);
void displayTempDetails(float, uint8_t, uint8_t, uint8_t);
float readVoltage(byte);
float getTemperature(uint8_t);
void displayVolume(void);
void displayMute(void);
void displayInput(void);
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

void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);
  
  SPI.begin();

  delay(2500);
  right_display.begin();
  right_display.setPowerSave(0);
  right_display.setFont(u8g2_font_cu12_tr); // u8g2_font_inr38_mf
  
  left_display.begin();
  left_display.setPowerSave(0);
  left_display.setFont(u8g2_font_cu12_tr); 

  
  
  Wire.begin();

  setupRotaryEncoders();

  
  relayController.begin();
  // Define all pins as OUTPUT and disable all relays
  for (byte pin = 0; pin <= 7; pin++)
  {
    relayController.pinMode(pin, OUTPUT);
    relayController.digitalWrite(pin, LOW);
  }
  
  muses.begin();

  //- ads1115.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  //- ads1115.begin();
  /*
  if (!ads1115.begin())
  {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
  */ 

  // Start IR reader
  irrecv.enableIRIn();

  // Read setting from EEPROM
  readSettingsFromEEPROM();
  readRuntimeSettingsFromEEPROM();

  // Check if settings stored in EEPROM are INVALID - if so, we write the default settings to the EEPROM and continue with those
  if ((Settings.Version != (float)VERSION) || (RuntimeSettings.Version != (float)VERSION))
  {
    right_display.clearBuffer();
    right_display.drawStr(0, 63, "Reset");  // 0 left, 0 top bottom appx 100
    right_display.sendBuffer();
    delay(2000);
    writeDefaultSettingsToEEPROM();
  }
  else
  {
    right_display.clearBuffer();
    right_display.drawStr(0, 63, "Eeprom ok");  // 0 left, 0 top bottom appx 100
    right_display.sendBuffer();
    delay(2000);
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
  /*oled.lcdOn();
  oled.clear();

  // Turn on Mezmerize B1 Buffer via power on/off relay
  if (Settings.ExtPowerRelayTrigger)
  {
    digitalWrite(POWER_RELAY_PIN, HIGH);
  }

  // The controller is now ready - save the timestamp
  mil_On = millis();

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
  appMode = APP_NORMAL_MODE;
  // Keep start volume for current input lower than max allowed start volume
//  RuntimeSettings.InputLastVol[RuntimeSettings.CurrentInput] = minimum(RuntimeSettings.InputLastVol[RuntimeSettings.CurrentInput], Settings.MaxStartVolume); // Avoid setting volume higher than MaxStartVol
//  setInput(RuntimeSettings.CurrentInput);
//  displayTemperatures();
//  unmute();

  UIkey = KEY_NONE;
  lastReceivedInput = KEY_NONE;
}

void loop() {
  //For test purposes
  /*
  for (byte pin = 0; pin <= 7; pin++)
  {
    relayController.pinMode(pin, OUTPUT);
    relayController.digitalWrite(pin, HIGH);
  }
  digitalWrite(POWER_CONTROL_PIN, HIGH);
  delay(2000);
  */
  for (byte pin = 0; pin <= 7; pin++)
  {
    relayController.pinMode(pin, OUTPUT);
    relayController.digitalWrite(pin, LOW);
  }
  digitalWrite(POWER_CONTROL_PIN, LOW);

  /*
  for (int i=0; i < 240; i++) {
    right_display.clearBuffer();
    right_display.drawStr(i, 63, "Let's");  // 0 left, 0 top bottom appx 100
    right_display.sendBuffer();
    left_display.clearBuffer();
    left_display.drawStr(i, 63, "rock");  // 0 left, 0 top bottom appx 100
    left_display.sendBuffer();
  }
  */

  // Display logo
  right_display.clearBuffer();
  right_display.drawXBMP(77, 0, 130, 64, thePreAmpLogo);
  right_display.sendBuffer();
  delay(2000);

  /*
  int16_t adc0, adc1, adc2, adc3;
  float volts0, volts1, volts2, volts3;
 
  adc0 = ads1115.readADC_SingleEnded(0);
  adc1 = ads1115.readADC_SingleEnded(1);
  adc2 = ads1115.readADC_SingleEnded(2);
  adc3 = ads1115.readADC_SingleEnded(3);
 
  volts0 = ads1115.computeVolts(adc0);
  volts1 = ads1115.computeVolts(adc1);
  volts2 = ads1115.computeVolts(adc2);
  volts3 = ads1115.computeVolts(adc3);
  
  left_display.clearBuffer();
  left_display.setCursor(1, 63);
  left_display.print(volts0);
  left_display.sendBuffer();
  delay(2000);
  */

  for (int i=0; i < 100; i++) {
    if (irrecv.decode(&IRresults)) {
      right_display.clearBuffer();
      right_display.drawStr(0, 63, uint64ToString(IRresults.value, HEX).c_str());  // 0 left, 0 top bottom appx 100
      right_display.sendBuffer();
      irrecv.resume();  // Receive the next value
    }
    else 
    {
      right_display.clearBuffer();
      right_display.drawStr(0, 63, "No IR received");  // 0 left, 0 top bottom appx 100
      right_display.sendBuffer();
    }
     left_display.clearBuffer();
    left_display.setCursor(1, 63);
    left_display.print("Vol 0");
    left_display.sendBuffer();
    setVolume(0);
    delay(1000);
    left_display.clearBuffer();
    left_display.setCursor(1, 63);
    left_display.print("Vol 200");
    left_display.sendBuffer();
    setVolume(200);
    delay(1000);
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
  Settings.IR_UP = 0x0;
  Settings.IR_DOWN = 0x0;
  Settings.IR_REPEAT = 0x0;
  Settings.IR_LEFT = 0x0;
  Settings.IR_RIGHT = 0x0;
  Settings.IR_SELECT = 0x0;
  Settings.IR_BACK = 0x0;
  Settings.IR_MUTE = 0x0;
  Settings.IR_PREVIOUS = 0x00;
  Settings.IR_ONOFF = 0x0;
  Settings.IR_1 = 0x0;
  Settings.IR_2 = 0x0;
  Settings.IR_3 = 0x0;
  Settings.IR_4 = 0x0;
  Settings.IR_5 = 0x0;
  Settings.IR_6 = 0x0;
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
  muses.setVolume(newVolumeStep);
}
