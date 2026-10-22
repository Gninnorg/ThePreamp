---
name: flash-preamp
description: "Use when: building the ThePreamp firmware, building the SPIFFS filesystem image (data/ folder), and uploading both to the preamp over WiFi (ElegantOTA) or USB. Keywords: build, flash, upload, uploadfs, OTA, firmware, filesystem, PlatformIO."
argument-hint: "[preamp host/IP, or 'usb']"
---

# Build and upload ThePreamp firmware + filesystem

Project: `Software\ThePreamp Controller` (PlatformIO, env `esp32doit-devkit-v1`, SPIFFS image built from `data\`).

## Steps

1. Determine the target: the preamp's IP/hostname (OTA) or `usb`. If not given, ask the user.
2. Run the script (it resolves the project folder itself, so it can be run from anywhere):

   ```powershell
   .\.github\skills\flash-preamp\flash.ps1 -Target <host-or-ip>   # OTA via ElegantOTA
   .\.github\skills\flash-preamp\flash.ps1 -Target usb            # serial upload
   ```

3. The script builds firmware (`pio run`), builds the filesystem (`pio run -t buildfs`), then uploads.
   - OTA: uploads the filesystem first, then the firmware (which reboots the device), using ElegantOTA's `/ota/start?mode=fs|fr` and `/ota/upload`.
   - USB: `pio run -t uploadfs` then `pio run -t upload`.
4. Report any build or upload error output. After success, remind the user to hard-refresh the browser (data files are served without cache headers).

## Notes
- Requires `pio` (PlatformIO CLI) on PATH; falls back to the VS Code PlatformIO install at `%USERPROFILE%\.platformio\penv\Scripts\pio.exe`, then `python -m platformio`.
- If OTA fails, check that `http://<host>/update` opens; the same page can be used for manual upload of `.pio\build\esp32doit-devkit-v1\firmware.bin` and `spiffs.bin`.
