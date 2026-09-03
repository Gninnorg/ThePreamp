#include "trigger_controller.h"
#include "controller_config.h"

void setTrigger1On()
{
  if (Settings.Trigger1Active)
  {
    relayController.digitalWrite(2, HIGH);
    if (Settings.Trigger1Type == 0)
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
    if (Settings.Trigger1Type == 0)
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
    if (Settings.Trigger2Type == 0)
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
    if (Settings.Trigger2Type == 0)
    {
      relayController.digitalWrite(1, HIGH);
      delay(200);
    }
    relayController.digitalWrite(1, LOW);
  }
}

void unmuteOutput()
{
  relayController.digitalWrite(0, HIGH);
}

void muteOutput()
{
  relayController.digitalWrite(0, LOW);
}
