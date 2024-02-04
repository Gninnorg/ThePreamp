#include <Arduino.h>
#include <SPI.h>
#include <U8g2lib.h>

/* ----- Harware SPI (SCL=18, SDA=23, SS=5) -----
OLED   =>    ESP32
GND    ->    GND
VCC    ->    3V3
SCL    ->    18 (CLK / SCL / SCLK)
SDA    ->    23 (MOSI / DIN / SDA)
RST     ->   22 (RST / RES/ Reset)
 DC    ->    21 (DC)
 CS     ->   5  (CS / SS)
*/

U8G2_SH1122_256X64_F_4W_HW_SPI display1(U8G2_R0, /* cs=*/ 5, /* dc=*/ 21, /* reset=*/ 22);
U8G2_SH1122_256X64_F_4W_HW_SPI display2(U8G2_R0, /* cs=*/ 32, /* dc=*/ 21, /* reset=*/ 33);

// put function declarations here:
int myFunction(int, int);

void setup() {
  display1.begin();
  display1.setFont(u8g2_font_inr38_mf);
  display2.begin();
  display2.setFont(u8g2_font_inr38_mf); // u8g2_font_ncenB08_tr
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