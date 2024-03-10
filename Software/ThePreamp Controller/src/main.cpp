#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

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

/* ----- I2C -----
GND    ->    GND
VCC    ->    3V3
SCL    ->    D22
SDA    ->    D21

*/

U8G2_SH1122_256X64_F_4W_HW_SPI display1(U8G2_R0, /* cs=*/ 12, /* dc=*/ 4, /* reset=*/ 32);
U8G2_SH1122_256X64_F_4W_HW_SPI display2(U8G2_R0, /* cs=*/ 13, /* dc=*/ 4, /* reset=*/ 33);

// put function declarations here:
int myFunction(int, int);

void setup() {
  Serial.begin(115200);

  display1.begin();
  display1.setFont(u8g2_font_inr38_mf);
  display2.begin();
  display2.setFont(u8g2_font_inr38_mf); // u8g2_font_ncenB08_tr

  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  if (!ads.begin())
  {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
   
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i=0; i < 240; i++) {
    display1.clearBuffer();
    display1.drawStr(i, 63, "Let's");  // 0 left, 0 top bottom appx 100
    display1.sendBuffer();
    display2.clearBuffer();
    display2.drawStr(i, 63, "rock");  // 0 left, 0 top bottom appx 100
    display2.sendBuffer();
  }
  
  int16_t adc0, adc1, adc2, adc3;
  float volts0, volts1, volts2, volts3;
 
  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);
 
  volts0 = ads.computeVolts(adc0);
  volts1 = ads.computeVolts(adc1);
  volts2 = ads.computeVolts(adc2);
  volts3 = ads.computeVolts(adc3);
  
  display2.clearBuffer();
  display2.setCursor(1, 63);
  display2.print(volts0);
  display2.sendBuffer();
  
  for (int i=0; i < 256; i++) {
    display1.clearBuffer();
    display1.setCursor(i, 63);
    display1.print(i);
    display1.sendBuffer();
  }
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}