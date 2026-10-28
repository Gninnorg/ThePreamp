# ThePreamp Controller

Firmware for an ESP32-based controller for "ThePreAmp" — a Muses72323-based audio preamplifier. Handles volume/balance/input control via rotary encoders and IR remote, two dual-OLED displays, relay-based input switching and trigger outputs, and a Wi-Fi web UI (settings, remote control, OTA firmware/filesystem updates).

## Hardware overview

- **MCU**: ESP32 (`esp32doit-devkit-v1`), see [platformio.ini](platformio.ini).
- **Volume/balance control**: Muses72323 digital attenuator over SPI ([lib/Muses72323](lib/Muses72323)).
- **Displays**: two `U8G2_SH1122_256X64` OLEDs over hardware SPI (left = input name / status, right = volume / signal / temperature).
- **Input switching / triggers / output mute**: `Adafruit_MCP23008` I²C GPIO expander (`relayController`).
- **User input**: two rotary encoders with push buttons ([lib/ClickEncoder](lib/ClickEncoder)) plus an IR receiver (`IRremoteESP8266`).
- **Temperature sensing**: `Adafruit_ADS1115` ADC reading two NTC channels.
- **Persistence**: settings and runtime state are stored in an external I²C EEPROM (`extEEPROM`).

## Application modes

Defined by `AppModeValues` in [include/controller_config.h](include/controller_config.h) and driven from `loop()` in [src/main.cpp](src/main.cpp):

- `APP_NORMAL_MODE` — normal operation: volume, input selection, mute, screen saver, standby timers.
- `APP_BALANCE_MODE` — entered via `KEY_SELECT`; `KEY_UP`/`KEY_DOWN` adjust balance in 0.25 dB steps up to ±6 dB (`BALANCE_MAX_OFFSET`), `KEY_SELECT` again saves it and returns to normal mode.
- `APP_STANDBY_MODE` — entered via `KEY_OFF` or an inactivity/temperature timeout; only `KEY_ON` (double-click of encoder 2, or an IR "on" code) is handled, which re-runs `startUp()`.

## Startup / trigger sequence

`startUp()` in [src/main.cpp](src/main.cpp) runs on power-up and whenever the unit wakes from standby:

1. Shows the boot logo, (re)connects Wi-Fi if needed, and turns on the external power relay (`POWER_CONTROL_PIN`) if `Settings.ExtPowerRelayTrigger` is set.
2. Stamps `mil_On = millis()` as the startup reference time.
3. For each of the two triggers, if `TriggerXActive` is set, computes a target fire time `mil_On + TriggerXOnDelay * 1000`; inactive triggers are skipped entirely (never delayed, turned on, or shown on screen).
4. Blocks in a loop until every active trigger has fired:
   - Turns a trigger on (`setTriggerXOn()`, see [src/trigger_controller.cpp](src/trigger_controller.cpp)) once its target time is reached.
   - Every 100 ms, updates both displays via `displayTriggerCountdown()` ([src/display_controller.cpp](src/display_controller.cpp)): shows `"Wait... N"` while counting down, `"On"` once fired, or nothing for a disabled trigger.
5. Selects the current input, clamps its startup volume to `MaxStartVolume`, unmutes (standby leaves the mute flag set, so `unmute()` re-applies the volume and updates the display), enables the output relay, and refreshes both displays.

Trigger on/off (`setTrigger1/2On/Off` in [src/trigger_controller.cpp](src/trigger_controller.cpp)) always checks `TriggerXActive` first (a no-op if disabled), and honors `TriggerXType`: `0` = pulsed (200 ms pulse then release), otherwise latched (stays energized). Triggers are turned off in `toStandbyMode()` in [src/input_controller.cpp](src/input_controller.cpp).

## User input handling

[src/input_controller.cpp](src/input_controller.cpp) polls the two rotary encoders (turn = volume/balance, click = select/mute, double-click = on/off) and the IR receiver, normalizing everything into a single `UIkey` value (`KEY_UP`, `KEY_DOWN`, `KEY_LEFT/RIGHT`, `KEY_1`-`KEY_5`, `KEY_MUTE`, `KEY_SELECT`, `KEY_ON`/`KEY_OFF`, etc.) consumed by `loop()`. The `IR_POWER` code toggles power (on if in standby, off otherwise) independently of the dedicated `IR_ON`/`IR_OFF` codes.

## Audio control

[src/audio_controller.cpp](src/audio_controller.cpp) drives the Muses72323 attenuator: `setVolume()`, `mute()`/`unmute()`, `setInput()`/`setPrevInput()`/`setNextInput()` (switches the MCP23008 input-select relays and applies each input's stored gain/volume/balance limits), and balance adjustment (`changeBalance()`, `adjustBalance()`, `saveBalance()`, `applyBalance()`).

## Displays

[src/display_controller.cpp](src/display_controller.cpp):

- `left_display_update()` — shows the selected input name (or "Set balance" while adjusting balance), if `Settings.DisplaySelectedInput` is enabled.
- `right_display_update()` — shows volume (steps or dB) or a speaker-with-cross mute icon (drawn with primitives, no text), Wi-Fi signal strength, and temperature bars, depending on settings.
- `displayBalance()` — full-screen balance adjustment UI with a bar graph.
- `displayTriggerCountdown()` — startup trigger countdown/status (see above).
- `ScreenSaverOn()`/`ScreenSaverOff()` — blanks/restores both displays after `Settings.DisplayTimeout` seconds of inactivity, if `Settings.ScreenSaverActive`.

### Shared SPI bus

Both displays and the Muses72323 share SCK/MOSI (and the displays share DC); only their CS pins differ. Two rules keep them from showing each other's data:

- At the very start of `setup()` the display CS pins are pulled up and all CS and reset pins are driven high, so no device listens in before U8g2 initializes it.
- Only the main `loop()` task may use the displays or the Muses chip. Web handlers run on the AsyncTCP task and must never call `setInput()`, `setVolume()`, `mute()`/`unmute()` or any `*_display_update()` directly (two tasks transferring at once leaves both CS lines low, which mirrors one display onto the other). They queue a request instead — see below.

## Settings persistence

[src/controller_config.cpp](src/controller_config.cpp) defines `Settings` (`mySettings`, persistent configuration — network, volume limits, per-input config, IR codes, triggers, display options) and `RuntimeSettings` (`myRuntimeSettings`, current input/volume/mute/balance memory), both stored in and loaded from external EEPROM. `exportSettingsAsJson()` serializes `Settings` to JSON (ArduinoJson 7 API: `JsonDocument`) for the web UI and WebSerial console. On boot, if the stored `Version` doesn't match the firmware's `VERSION`, defaults are written (EEPROM layout changed).

## Network endpoints

Set up in [src/wifi_support.cpp](src/wifi_support.cpp). Two modes:

### Station mode (Wi-Fi configured, `setupNormalModeServer()`)

**Pages (served from SPIFFS, see [data/](data)):**

| Path | Method | Description |
|---|---|---|
| `/` , `/remote` | GET | [remote.html](data/remote.html) — remote-control web UI |
| `/settings.html` (via static handler) | GET | [settings.html](data/settings.html) — full settings editor UI |
| `/update.html` (via static handler) | GET | [update.html](data/update.html) — firmware/filesystem update helper page |
| any other static asset in `data/` | GET | served as-is via `server.serveStatic("/", SPIFFS, "/")` |

**JSON API:**

| Path | Method | Description |
|---|---|---|
| `/api/settings` | GET | Returns the full `Settings` structure as JSON (`exportSettingsAsJson()`) |
| `/api/settings` | POST | Body: `settings=<url-encoded JSON>`. Validates and applies each field, then writes to EEPROM |
| `/api/remote` | GET | Returns current input, volume, mute state, standby state, and per-input name/limits as JSON |
| `/api/remote` | POST | Body params `input`, `volume`, `mute=toggle` and/or `power=toggle` (form-encoded). Invalid values return 400. Changes are queued for `loop()` (see below) and the returned state already reflects them |

**Legacy simple GET endpoints** (plain-text responses, used by e.g. simple remote/automation integrations):

| Path | Method | Description |
|---|---|---|
| `/INPUT1` … `/INPUT5` | GET | Queues selection of input 0–4; returns `1`, or `0` if the input is inactive or the unit is in standby |
| `/MUTE` | GET | Queues mute |
| `/UNMUTE` | GET | Queues unmute |

Requests are ignored while not in normal mode (e.g. standby). Unknown paths return 404.

**Request queue.** The web handlers only validate and store the request in `requestedInput`, `requestedVolume` and `requestedMute` (declared in [include/input_controller.h](include/input_controller.h)); `processPendingRequests()` in [src/main.cpp](src/main.cpp) applies them from `loop()`, like `requestedPowerCommand` does for power. MQTT commands already run inside `loop()`.

**Firmware/OTA & debug:**

| Path | Method | Description |
|---|---|---|
| `/update` (ElegantOTA) | GET/POST | Web-based firmware and filesystem OTA update UI, mounted via `ElegantOTA.begin(&server)` |
| `/webserial` (WebSerial) | GET/WS | Browser-based serial console, mounted via `WebSerial.begin(&server)`; supports a small command set (`HELP`, `EXPORT-SETTINGS`) via `WebSerial.onMessage()` |

### Access-point mode (no Wi-Fi configured yet, `setupAccessPointServer()`)

Broadcasts an open AP named `ThePreAmp` with a captive-portal-style DNS server (`dnsServer`, redirects all lookups to itself).

| Path | Method | Description |
|---|---|---|
| `update.html` | GET | [update.html](data/update.html) |
| `/` | POST | Accepts `ssid`, `pass`, `ip`, `gateway` form fields, saves them to EEPROM, and restarts the device |
| any unmatched path (`onNotFound`) | GET | Serves [wifi.html](data/wifi.html) — Wi-Fi setup form, styled like the remote page with inline CSS (also shown as a QR code on the left display) |
| `/update` (ElegantOTA) | GET/POST | Same OTA UI as above |

## Client-side gotcha

[data/settings.html](data/settings.html)'s `payload()` function must convert every non-checkbox form field (including `<select>` elements, whose `type` is `select-one`) to a JSON number before POSTing — the server's `boundedByte()` in [src/wifi_support.cpp](src/wifi_support.cpp) only accepts numeric JSON values and silently keeps the previous setting otherwise. After editing `data/`, remember to rebuild **and re-upload** the SPIFFS image (`uploadfs`), and hard-refresh the browser (`data/` files are served without cache-control headers, so browsers may serve a stale cached copy).

## Build, flash and test

- **Build/flash:** use the `flash-preamp` skill script [../../.github/skills/flash-preamp/flash.ps1](../../.github/skills/flash-preamp/flash.ps1) (`-Target <ip>` for OTA or `-Target usb`). It rebuilds and uploads firmware and filesystem only when their inputs changed (state is kept in `.pio/flash-state/`); `-Force` overrides this.
- **Test:** [tools/test-preamp.ps1](tools/test-preamp.ps1) exercises the web API of a running unit (pages, mute, volume, inputs, settings/MQTT endpoints) and restores the previous state; `-MqttHost` adds an MQTT mute test. `-IncludeDisplay` adds webcam checks of what the two displays show (input name, volume, mute, indicators, screen saver; needs a USB webcam, Python/OpenCV and Tesseract, see the [webcam skill](.github/skills/preamp-webcam-display-test/SKILL.md)). [tools/TESTING.md](tools/TESTING.md) is a manual checklist for what a script can't see (relays, audio, IR, balance/info screens).

## MQTT / Home Assistant

[src/mqtt_controller.cpp](src/mqtt_controller.cpp) connects to an MQTT broker (PubSubClient, handled from `loop()`) and publishes Home Assistant MQTT discovery for four entities: **Power** (switch), **Mute** (switch), **Volume** (number, 0..`VolumeSteps`) and **Input** (select, using the configured input names).

Configuration is stored in NVS (not the EEPROM settings). Set it in the **MQTT / Home Assistant** section of the web settings page ([data/settings.html](data/settings.html)) or via the web API:

```
curl -X POST http://<preamp-ip>/api/mqtt -d enabled=true -d host=192.168.1.10 -d port=1883 -d user=ha -d pass=secret -d baseTopic=thepreamp
curl http://<preamp-ip>/api/mqtt     # current config + connection state
```

Topics (default base `thepreamp`): `<base>/status` (`online`/`offline`, LWT), `<base>/state` (retained JSON: `power`, `volume`, `mute`, `input`, `input_index`), and commands on `<base>/power/set` (`ON`/`OFF`), `<base>/mute/set` (`ON`/`OFF`), `<base>/volume/set` (step number) and `<base>/input/set` (input name). Mute/volume/input commands are ignored while in standby.