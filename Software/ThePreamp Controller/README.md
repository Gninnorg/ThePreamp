# ThePreamp Controller

## Trigger logic

Triggers are used to switch external devices (e.g. power amplifiers) on/off together with the preamp. There are two triggers, each wired to a relay on the `relayController` (MCP23008):

- Trigger 1 relay -> MCP23008 pin 2
- Trigger 2 relay -> MCP23008 pin 1

Each trigger has its own settings, stored in `Settings` (see [include/controller_config.h](include/controller_config.h)):

- `TriggerXActive` — whether the trigger is enabled at all.
- `TriggerXType` — `0` = pulsed relay (briefly energized then released), otherwise a latching/steady relay that stays energized.
- `TriggerXOnDelay` — number of seconds to wait after startup before turning the trigger on.
- `TriggerXTemp` — temperature threshold used to force standby (not related to the on-delay).

### Turning a trigger on/off

Implemented in [src/trigger_controller.cpp](src/trigger_controller.cpp):

- `setTrigger1On()` / `setTrigger2On()` — only act if the trigger is active. Drives the relay pin `HIGH`; if `TriggerXType == 0` (pulsed), it is pulsed for 200 ms and released back to `LOW`, otherwise it stays `HIGH`.
- `setTrigger1Off()` / `setTrigger2Off()` — mirror logic to release/turn off the relay.

### Startup countdown sequence

Implemented in `startUp()` in [src/main.cpp](src/main.cpp), run both on power-up and when waking from standby (`KEY_ON`):

1. `mil_On` is stamped with the current `millis()` as the startup reference time.
2. For each trigger, a target fire time is computed: `mil_On + TriggerXOnDelay * 1000` — but only if the trigger is active. If a trigger is inactive, its delay is `0` and it is skipped entirely (never turned on by this sequence, never shown on the display).
3. A blocking loop runs as long as at least one trigger still has a pending (non-zero) delay:
   - If a trigger's target time has passed, it is turned on (`setTriggerXOn()`) and its delay is cleared to `0`, marking it as fired.
   - While any trigger is still pending, `displayTriggerCountdown()` (see [src/display_controller.cpp](src/display_controller.cpp)) is called every 100 ms to update the on-screen countdown.
   - The loop is blocking by design (matches the original startup behavior): normal input handling, OTA, etc. do not run until every active trigger has fired.
4. Once both triggers are resolved (fired or inactive), the loop exits and normal startup continues (input selection, unmuting output, refreshing the displays).

### On-screen countdown display

`displayTriggerCountdown(int trigger1SecondsRemaining, int trigger2SecondsRemaining)`:

- Left display shows the status for trigger 1, right display shows the status for trigger 2.
- Each parameter uses a small set of sentinel values:
  - `-1` — nothing to show (trigger is inactive), the display area is left blank.
  - `-2` — trigger has fired, shows `"On"`.
  - `>= 0` — seconds remaining until the trigger fires, shows `"Wait... N"`.

### Turning triggers off

Triggers are turned off in `toStandbyMode()` (see [src/input_controller.cpp](src/input_controller.cpp)), which unconditionally calls `setTrigger1Off()` and `setTrigger2Off()` (each is a no-op if that trigger isn't active).
