#include "input_controller.h"
#include "audio_controller.h"
#include "display_controller.h"
#include "trigger_controller.h"

extern Muses72323 muses;
extern Adafruit_MCP23008 relayController;
extern byte appMode;
extern unsigned long mil_LastUserInput;
extern unsigned long last_KEY_ONOFF;
extern const long interval;

byte getUserCommand()
{
  if (interruptCounter > 0)
  {
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
    totalInterruptCounter++;
  }

  byte receivedInput = KEY_NONE;

  e1value += encoder1->getValue();
  if (e1value != e1last)
  {
    if (e1value > e1last)
      receivedInput = KEY_UP;
    if (e1value < e1last)
      receivedInput = KEY_DOWN;
    e1last = e1value;
  }

  button1 = encoder1->getButton();
  switch (button1)
  {
  case ClickEncoder::Clicked:
    receivedInput = KEY_SELECT;
    break;
  default:
    break;
  }

  e2value += encoder2->getValue();
  if (e2value != e2last)
  {
    if (e2value > e2last)
      receivedInput = KEY_RIGHT;
    if (e2value < e2last)
      receivedInput = KEY_LEFT;
    e2last = e2value;
  }

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

  if (irrecv.decode(&IRresults))
  {
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
    irrecv.resume();
  }

  if (receivedInput != KEY_NONE)
  {
    mil_LastUserInput = millis();
  }

  return receivedInput;
}

void toAppNormalMode()
{
  left_display_update();
  right_display_update();
  appMode = APP_NORMAL_MODE;
}

void toStandbyMode()
{
  appMode = APP_STANDBY_MODE;
  writeRuntimeSettingsToEEPROM();
  mute();
  unmuteOutput();
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

boolean setInput(uint8_t NewInput)
{
  boolean result = false;
  if (Settings.Input[NewInput].Active != INPUT_INACTIVATED && NewInput >= 0 && NewInput <= 4 && appMode == APP_NORMAL_MODE)
  {
    if (!RuntimeSettings.Muted)
      mute();

    relayController.digitalWrite(7 - RuntimeSettings.CurrentInput, LOW);
    RuntimeSettings.PrevSelectedInput = RuntimeSettings.CurrentInput;
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

void setPrevInput(void)
{
  byte nextInput = (RuntimeSettings.CurrentInput == 0) ? 4 : RuntimeSettings.CurrentInput - 1;
  while (Settings.Input[nextInput].Active == INPUT_INACTIVATED)
  {
    nextInput = (nextInput == 0) ? 5 : nextInput - 1;
  }
  setInput(nextInput);
}

void setNextInput(void)
{
  byte nextInput = (RuntimeSettings.CurrentInput == 4) ? 0 : RuntimeSettings.CurrentInput + 1;
  while (Settings.Input[nextInput].Active == INPUT_INACTIVATED)
  {
    nextInput = (nextInput == 4) ? 0 : nextInput + 1;
  }
  setInput(nextInput);
}

bool changeBalance()
{
  bool complete = false;
  bool result = false;
  byte OldValue = RuntimeSettings.InputLastBal[RuntimeSettings.CurrentInput];
  byte NewValue = RuntimeSettings.InputLastBal[RuntimeSettings.CurrentInput];

  appMode = APP_BALANCE_MODE;

  return result;
}
