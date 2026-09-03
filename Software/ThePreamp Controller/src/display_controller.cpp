#include "display_controller.h"
#include "controller_config.h"
#include "audio_controller.h"
#include "logo.h"
#include <WiFi.h>

extern unsigned long mil_LastUserInput;

void displayLogo(void)
{
  left_display.clearBuffer();
  left_display.drawXBMP(77, 0, 130, 64, thePreAmpLogo);
  left_display.sendBuffer();

  right_display.clearBuffer();
  right_display.sendBuffer();
  delay(1000);
}

void left_display_update(void)
{
  if (Settings.DisplaySelectedInput)
  {
    if (ScreenSaverIsOn)
      ScreenSaverOff();

    left_display.setFont(u8g2_font_inb42_mr);

    int16_t textWidth = left_display.getStrWidth(Settings.Input[RuntimeSettings.CurrentInput].Name);
    int16_t xPos = (256 - textWidth) / 2;
    int16_t yPos = 52;
    left_display.clearBuffer();
    left_display.drawStr(xPos, yPos, Settings.Input[RuntimeSettings.CurrentInput].Name);
    left_display.sendBuffer();
  }
}

void right_display_update(void)
{
  right_display.clearBuffer();

  if (Settings.DisplayVolume)
  {
    right_display.setFont(u8g2_font_inb63_mn);
    if (!RuntimeSettings.Muted)
    {
      if (Settings.DisplayVolume == 1)
      {
        char buffer[10];
        snprintf(buffer, sizeof(buffer), "%d", RuntimeSettings.CurrentVolume);
        int16_t textWidth = right_display.getStrWidth(buffer);
        int16_t xPos = (256 - textWidth) / 2;
        int16_t yPos = 63;
        right_display.drawStr(xPos, yPos, buffer);
      }
      else
      {
        char buffer[10];
        snprintf(buffer, sizeof(buffer), "%d", (calculateAttenuation(RuntimeSettings.CurrentVolume, Settings.VolumeSteps, Settings.MinAttenuation, Settings.MaxAttenuation) / 4));
        int16_t textWidth = right_display.getStrWidth(buffer);
        int16_t xPos = (256 - textWidth) / 2;
        int16_t yPos = 63;
        right_display.drawStr(xPos, yPos, buffer);
      }
    }
    else
    {
      int16_t textWidth = right_display.getStrWidth("MUTE");
      int16_t xPos = (256 - textWidth) / 2;
      int16_t yPos = 63;
      right_display.drawStr(xPos, yPos, "MUTE");
    }
  }

  switch (WiFi.status())
  {
    case WL_CONNECTED:
      drawSignalStrength(WiFi.RSSI());
      break;
  }

  if (Settings.DisplayTemperature1 || Settings.DisplayTemperature2)
  {
    drawTemperatureMeasurements();
  }

  right_display.sendBuffer();
  if (ScreenSaverIsOn)
      ScreenSaverOff();
}

void displayBalance(byte Value)
{
  /*
  // Balance display stub kept in display module for future implementation.
  // Original logic was previously in the input controller.
  */
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

void drawTemperatureMeasurements(void)
{
  right_display.drawFrame(232,34,24,14);
  right_display.drawFrame(232,50,24,14);

  right_display.setFontMode(1);
  right_display.setDrawColor(1);

  right_display.drawFrame(232,34,24,14);
  int tempRight = static_cast<int>(getTemperature(0));
  right_display.drawBox(234,36,map(tempRight, 0, 65, 0, 20),10);

  right_display.drawFrame(232,50,24,14);
  int tempLeft = static_cast<int>(getTemperature(1));
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

void ScreenSaverOn(void)
{
  ScreenSaverIsOn = true;
  left_display.clearDisplay();
  right_display.clearDisplay();
}

void ScreenSaverOff(void)
{
  if (ScreenSaverIsOn)
  {
    ScreenSaverIsOn = false;
    left_display_update();
    right_display_update();
  }
}
