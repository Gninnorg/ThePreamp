#include "audio_controller.h"
#include "controller_config.h"
#include "display_controller.h"
#include "trigger_controller.h"

extern Muses72323 muses;
extern byte appMode;

int calculateAttenuation(byte logicalStep, byte maxLogicalSteps, byte minAttenuation_dB, byte maxAttenuation_dB)
{
  if (minAttenuation_dB >= maxAttenuation_dB ||
      logicalStep < 1 ||
      logicalStep > maxLogicalSteps ||
      maxLogicalSteps < 10 ||
      maxLogicalSteps <= ((maxAttenuation_dB - minAttenuation_dB) / 4))
    return minAttenuation_dB * -4;

  float attenuationRange = maxAttenuation_dB - minAttenuation_dB;
  float attenuationPerStep = attenuationRange / (maxLogicalSteps - 1);
  float attenuation = maxAttenuation_dB - (logicalStep - 1) * attenuationPerStep;
  attenuation = round(attenuation * 4) / 4;
  int volumeStep = static_cast<int>(attenuation * -4);
  return volumeStep;
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
      int CurrentAttenuation = calculateAttenuation(RuntimeSettings.CurrentVolume, Settings.VolumeSteps, Settings.MinAttenuation, Settings.MaxAttenuation);
      if (Settings.Input[RuntimeSettings.CurrentInput].Active != INPUT_HT_PASSTHROUGH)
        RuntimeSettings.CurrentVolume = newVolumeStep;
      else
        RuntimeSettings.CurrentVolume = Settings.Input[RuntimeSettings.CurrentInput].MaxVol;
      RuntimeSettings.InputLastVol[RuntimeSettings.CurrentInput] = RuntimeSettings.CurrentVolume;

      int NewAttenuation = calculateAttenuation(RuntimeSettings.CurrentVolume, Settings.VolumeSteps, Settings.MinAttenuation, Settings.MaxAttenuation);
      if (NewAttenuation > CurrentAttenuation) {
        for (int i = CurrentAttenuation; i < NewAttenuation; i++) {
          muses.setVolume(i, i);
        }
      } else {
        if (CurrentAttenuation == NewAttenuation) {
          muses.setVolume(NewAttenuation, NewAttenuation);
        } else {
          for (int i = CurrentAttenuation; i > NewAttenuation; i--) {
            muses.setVolume(i, i);
          }
        }
      }
    }
    if (appMode == APP_NORMAL_MODE)
      right_display_update();
  }
}

void mute()
{
  if (Settings.MuteLevel)
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
