#include "display_controller.h"
#include "controller_config.h"
#include "audio_controller.h"
#include "logo.h"
#include <WiFi.h>

extern unsigned long mil_LastUserInput;
extern byte appMode;

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
  if (ScreenSaverIsOn)
    ScreenSaverOff();

  if (appMode == APP_BALANCE_MODE)
  {
    left_display.setFont(u8g2_font_inb24_mf);
    const char *text = "Set balance";
    int16_t textWidth = left_display.getStrWidth(text);
    int16_t xPos = (256 - textWidth) / 2;
    left_display.clearBuffer();
    left_display.drawStr(xPos, 40, text);
    left_display.sendBuffer();
    return;
  }

  if (Settings.DisplaySelectedInput)
  {
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

  // HT passthrough runs at a fixed level set by the AV receiver, so showing a volume value would be misleading
  bool isHtPassthrough = Settings.Input[RuntimeSettings.CurrentInput].Active == INPUT_HT_PASSTHROUGH;

  if (RuntimeSettings.Muted)
  {
    right_display.setFont(u8g2_font_inb63_mn);
    int16_t textWidth = right_display.getStrWidth("MUTE");
    int16_t xPos = (256 - textWidth) / 2;
    int16_t yPos = 63;
    right_display.drawStr(xPos, yPos, "MUTE");
  }
  else if (Settings.DisplayVolume && !isHtPassthrough)
  {
    right_display.setFont(u8g2_font_inb63_mn);
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
  right_display.clearBuffer();
  right_display.setDrawColor(1);

  int balanceOffset = (int)Value - BALANCE_CENTER;
  char label[16];
  if (balanceOffset == 0)
    strcpy(label, "C");
  else
    snprintf(label, sizeof(label), "%c %.2fdB", balanceOffset < 0 ? 'L' : 'R', getBalanceAttenuationDb(Value));

  right_display.setFont(u8g2_font_inb24_mf);
  int16_t textWidth = right_display.getStrWidth(label);
  right_display.drawStr((256 - textWidth) / 2, 26, label);

  const int16_t barX = 8;
  const int16_t barY = 40;
  const int16_t barWidth = 240;
  const int16_t barHeight = 14;
  const int16_t centerX = barX + barWidth / 2;

  // Bold outer frame so the scale reads well from a distance
  right_display.drawFrame(barX, barY, barWidth, barHeight);
  right_display.drawFrame(barX + 1, barY + 1, barWidth - 2, barHeight - 2);

  // Fill from center to the current position to show the deviation at a glance
  int16_t indicatorX = constrain(centerX + (int16_t)(((long)balanceOffset * (barWidth / 2 - 3)) / BALANCE_MAX_OFFSET), barX + 3, barX + barWidth - 3);
  int16_t fillX = min(centerX, indicatorX);
  int16_t fillWidth = abs(indicatorX - centerX);
  if (fillWidth > 0)
    right_display.drawBox(fillX, barY + 2, fillWidth, barHeight - 4);

  // Center marker, drawn taller than the bar so it stays visible
  right_display.drawBox(centerX - 1, barY - 4, 3, barHeight + 8);

  // Position marker, wider still for maximum visibility
  right_display.drawBox(indicatorX - 4, barY - 4, 8, barHeight + 8);

  right_display.sendBuffer();
}

void displayTriggerCountdown(int trigger1SecondsRemaining, int trigger2SecondsRemaining)
{
  char buffer[24];

  // Left display reflects trigger 1, right display reflects trigger 2
  left_display.clearBuffer();
  left_display.setFont(u8g2_font_inb24_mf);
  if (trigger1SecondsRemaining != -1)
  {
    if (trigger1SecondsRemaining == -2)
      strcpy(buffer, "On");
    else
      snprintf(buffer, sizeof(buffer), "Wait... %d", trigger1SecondsRemaining);
    int16_t textWidth = left_display.getStrWidth(buffer);
    left_display.drawStr((256 - textWidth) / 2, 40, buffer);
  }
  left_display.sendBuffer();

  right_display.clearBuffer();
  right_display.setFont(u8g2_font_inb24_mf);
  if (trigger2SecondsRemaining != -1)
  {
    if (trigger2SecondsRemaining == -2)
      strcpy(buffer, "On");
    else
      snprintf(buffer, sizeof(buffer), "Wait... %d", trigger2SecondsRemaining);
    int16_t textWidth = right_display.getStrWidth(buffer);
    right_display.drawStr((256 - textWidth) / 2, 40, buffer);
  }
  right_display.sendBuffer();
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
