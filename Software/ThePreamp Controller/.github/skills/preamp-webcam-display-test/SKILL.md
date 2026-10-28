---
name: preamp-webcam-display-test
description: "Capture and verify ThePreamp's physical display output with a USB webcam. Use when testing display text, taking display photos, or running camera-assisted preamp tests."
---

# Preamp Webcam Display Testing

Use this workflow to capture the preamp's two displays with a webcam connected to the Windows test PC and optionally verify the rendered text through the automated test runner.

## Requirements

- The webcam must be connected to the PC and aimed at both displays.
- Install Python and the capture dependency from `requirements.txt` in this folder.
- Install Tesseract OCR and make `tesseract.exe` available on `PATH`.
- The preamp must be powered on, reachable on the network, and not in standby.

## Automated test

From the ThePreamp Controller project directory, run:

```powershell
.\tools\test-preamp.ps1 -Target <preamp-ip> -IncludeDisplay
```

If the webcam is not the first camera, select its zero-based index:

```powershell
.\tools\test-preamp.ps1 -Target <preamp-ip> -IncludeDisplay -CameraIndex 1
```

The test runner loads [display-tests.ps1](../../../tools/display-tests.ps1) and, for each scenario, sets up the preamp through the web API, captures a frame with [capture_display.py](./scripts/capture_display.py) `--analyze`, and checks it by OCR (Tesseract) and by counting lit pixels in regions of each display. Scenarios:

- input name (left display) and volume (right display) for every active input; HT passthrough shows no volume
- volume as steps, as dB and hidden (`DisplayVolume` 1, 2, 0)
- mute icon (remote/web and `/MUTE`, `/UNMUTE`)
- WiFi and temperature indicators on/off
- screen saver: blanks after `DisplayTimeout`, wakes on activity, stays on when disabled
- `DisplaySelectedInput` off: no name on the left display
- with `-IncludePower`: displays blank in standby, "Wait... N" countdown (only if a trigger is active) and correct display after start-up

Settings changed by the scenarios are restored afterwards. A failure keeps the captured images (full frame, per-display OCR crops, annotated ROI overlay and JSON metrics) in a `thepreamp-display-<pid>` folder in the system temp directory.

Not covered because the API cannot trigger them: balance screen, info screens (IP/QR, build dates) and brightness (`DisplayOnLevel`/`DisplayDimLevel`).

## Capture and analysis options

`capture_display.py` requests 1920x1080 (`--width`/`--height`) and prints the resolution it actually got. `--analyze` writes `<output>-left.png`, `<output>-right.png`, `<output>-rois.png` and `<output>.json` (lit pixel counts for `left`, `right`, `right_main`, `right_center`, `right_wifi`, `right_temp`). The display regions are fractions of the frame (`--left-roi`, `--right-roi`, defaults in the script); if the camera is moved or zoomed, run with `--analyze` and adjust them until `-rois.png` shows the boxes on the displays.

Install the Python dependency from the ThePreamp Controller project directory with:

```powershell
python -m pip install -r .github\skills\preamp-webcam-display-test\requirements.txt
```

## Capture a picture only

```powershell
python .github\skills\preamp-webcam-display-test\scripts\capture_display.py --camera-index 0 --output "$env:TEMP\preamp-display.png"
```

Use `--help` for capture options. Camera framing, glare, focus, and exposure affect OCR reliability; position the camera so the displays are legible and avoid reflections.
