# Webcam-based checks of what the preamp shows on its two displays.
# Dot-sourced by test-preamp.ps1 when -IncludeDisplay is used, so it shares that script's
# $base, $CameraIndex, $TesseractPath, Check, Get-State and Send-Remote.
#
# Each check changes the preamp through its web API, captures the front panel with the webcam
# (see .github\skills\preamp-webcam-display-test) and reads each display with OCR or by counting
# lit pixels. Settings that are changed temporarily are restored afterwards.
#
# Not covered because they cannot be triggered through the web API (IR remote / encoder only):
# balance screen, info screens. DisplayOnLevel / DisplayDimLevel are stored but not used by the firmware.

$script:displayDir = Join-Path ([System.IO.Path]::GetTempPath()) "thepreamp-display-$PID"
$script:shotCounter = 0

# OCR confuses similar characters, so fold them together and drop everything but letters and digits
function Normalize-DisplayText([string]$text) {
    (($text.ToLower() -replace '[l1|]', 'i') -replace '[o]', '0') -replace '[^a-z0-9]', ''
}

function Read-DisplayText([string]$image, [switch]$Digits) {
    # Single digits are unreliable in line mode (7), so also try single-word (8) and single-character (10) modes
    $modes = if ($Digits) { @("7", "8", "10") } else { @("7") }
    $result = ""
    foreach ($mode in $modes) {
        $ocrArgs = @($image, "stdout", "--psm", $mode)
        if ($Digits) { $ocrArgs += @("-c", "tessedit_char_whitelist=0123456789-") }
        $text = & $TesseractPath @ocrArgs 2>$null
        $result = if ($null -eq $text) { "" } else { (($text -join " ") -replace "\s+", " ").Trim() }
        if (-not $Digits -or $result -match '[0-9]') { break }
    }
    return $result
}

# Captures the front panel. Returns the OCR text of each display and the lit-pixel counts, or $null on failure.
function Get-DisplayShot([string]$label) {
    New-Item -ItemType Directory -Force $script:displayDir | Out-Null
    $script:shotCounter++
    $image = Join-Path $script:displayDir ("{0:D2}-{1}.png" -f $script:shotCounter, ($label -replace '[^\w-]', '_'))
    & $script:pythonCommand.Source $script:cameraScript --camera-index $CameraIndex --output $image --analyze | Out-Null
    $metrics = [System.IO.Path]::ChangeExtension($image, ".json")
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path $metrics)) {
        Check "capture webcam image ($label)" $false "(camera index $CameraIndex)"
        return $null
    }
    $stem = $image.Substring(0, $image.Length - 4)
    [pscustomobject]@{
        Image       = $image
        Lit         = Get-Content $metrics -Raw | ConvertFrom-Json
        Left        = Read-DisplayText "$stem-left.png"
        Right       = Read-DisplayText "$stem-right.png"
        RightDigits = Read-DisplayText "$stem-right.png" -Digits
    }
}

function Describe-Shot($shot) {
    "(left='$($shot.Left)' right='$($shot.Right)' digits='$($shot.RightDigits)' lit: left=$($shot.Lit.left) right=$($shot.Lit.right) main=$($shot.Lit.right_main) center=$($shot.Lit.right_center) wifi=$($shot.Lit.right_wifi) temp=$($shot.Lit.right_temp); $($shot.Image))"
}

# Captures and runs the assertions on the result: Invoke-Shot "label" { param($s) Check ... }
function Invoke-Shot([string]$label, [scriptblock]$assertions) {
    $shot = Get-DisplayShot $label
    if ($null -ne $shot) { & $assertions $shot }
}

function Set-DeviceSettings([hashtable]$values) {
    Invoke-RestMethod "$base/api/settings" -Method Post -Body @{ settings = ($values | ConvertTo-Json -Compress) } -TimeoutSec 10 | Out-Null
}

# Any remote request counts as user activity: it wakes the screen saver and redraws the displays
function Wake-Display { Send-Remote @{ volume = (Get-State).volume } | Out-Null }

function Test-Digits($shot, [string]$expected) {
    # Digits-only text may contain Q/O/D standing in for the slashed zero
    $read = (($shot.RightDigits -replace '[QODØ]', '0') -replace '[^0-9]', '')
    if ($read -eq $expected) { return $true }
    $plain = (($shot.Right -replace '[QODØ]', '0') -replace '[^0-9]', '')
    return ($plain -eq $expected)
}

function Test-LeftShows($shot, [string]$name) {
    (Normalize-DisplayText $shot.Left).Contains((Normalize-DisplayText $name))
}

# Mirrors the firmware: DisplayVolume = 2 shows the attenuation in whole dB
function Get-ExpectedAttenuationText([int]$step, $config) {
    $range = [int]$config.MaxAttenuation - [int]$config.MinAttenuation
    $attenuation = [int]$config.MaxAttenuation - ($step - 1) * ($range / ([int]$config.VolumeSteps - 1))
    $attenuation = [Math]::Round($attenuation * 4, [MidpointRounding]::AwayFromZero) / 4
    [string][int][Math]::Floor($attenuation)
}

function Invoke-DisplayTests {
    Write-Host "`n== Webcam display output =="
    $config = Invoke-RestMethod "$base/api/settings" -TimeoutSec 10
    $restore = @{
        DisplayVolume        = [int]$config.DisplayVolume
        DisplaySelectedInput = [bool]$config.DisplaySelectedInput
        DisplayTimeout       = [int]$config.DisplayTimeout
        ScreenSaverActive    = [bool]$config.ScreenSaverActive
        DisplayTemperature1  = [int]$config.DisplayTemperature1
        DisplayTemperature2  = [int]$config.DisplayTemperature2
    }

    try {
        # Known baseline; the long timeout keeps the screen saver from blanking the displays between checks
        Set-DeviceSettings @{ DisplayVolume = 1; DisplaySelectedInput = $true; DisplayTimeout = 255; ScreenSaverActive = $true; DisplayTemperature1 = 3; DisplayTemperature2 = 3 }
        if ((Get-State).muted) { Send-Remote @{ mute = "toggle" } | Out-Null }

        Write-Host "-- Input name and volume per input"
        $state = Get-State
        $volumeInput = $null
        for ($i = 0; $i -lt 5; $i++) {
            $inp = $state.inputs[$i]
            if (-not $inp.active) { continue }
            Send-Remote @{ input = $i } | Out-Null
            Start-Sleep -Seconds 3
            $volume = (Get-State).volume
            Invoke-Shot "input$i" {
                param($s)
                Check "input $i ($($inp.name)): left display shows the name" (Test-LeftShows $s $inp.name) (Describe-Shot $s)
                if ($inp.passthrough) {
                    Check "input $i (HT passthrough): right display shows no volume" ($s.Lit.right_main -lt 20) (Describe-Shot $s)
                }
                else {
                    Check "input $i ($($inp.name)): right display shows volume $volume" (Test-Digits $s "$volume") (Describe-Shot $s)
                }
            }
            if (-not $inp.passthrough -and $null -eq $volumeInput) { $volumeInput = $i }
        }

        if ($null -eq $volumeInput) {
            Write-Host "SKIP  No active input without HT passthrough: volume display checks skipped." -ForegroundColor Yellow
        }
        else {
            Send-Remote @{ input = $volumeInput } | Out-Null
            Start-Sleep -Seconds 3
            $range = $state.inputs[$volumeInput]
            $min = [int]$range.minVolume
            $max = [int]$range.maxVolume
            $mid = [int][Math]::Floor(($min + $max) / 2)

            Write-Host "-- Volume in steps (DisplayVolume = 1)"
            foreach ($v in $min, $mid, $max) {
                Send-Remote @{ volume = $v } | Out-Null
                Start-Sleep -Seconds 2
                Invoke-Shot "volume$v" { param($s) Check "right display shows volume step $v" (Test-Digits $s "$v") (Describe-Shot $s) }
            }

            Write-Host "-- Volume in dB (DisplayVolume = 2)"
            Set-DeviceSettings @{ DisplayVolume = 2 }
            foreach ($v in $mid, $max) {
                $expected = Get-ExpectedAttenuationText $v $config
                Send-Remote @{ volume = $v } | Out-Null
                Start-Sleep -Seconds 2
                Invoke-Shot "db$v" { param($s) Check "right display shows $expected dB for volume step $v" (Test-Digits $s $expected) (Describe-Shot $s) }
            }

            Write-Host "-- Volume hidden (DisplayVolume = 0)"
            Set-DeviceSettings @{ DisplayVolume = 0 }
            Wake-Display
            Start-Sleep -Seconds 2
            Invoke-Shot "volume-hidden" {
                param($s)
                Check "right display shows no volume" ($s.Lit.right_main -lt 20) (Describe-Shot $s)
                Check "left display still shows the input name" (Test-LeftShows $s $range.name) (Describe-Shot $s)
            }

            Set-DeviceSettings @{ DisplayVolume = 1 }
            Send-Remote @{ volume = $mid } | Out-Null
            Start-Sleep -Seconds 2

            Write-Host "-- Mute"
            Send-Remote @{ mute = "toggle" } | Out-Null
            Start-Sleep -Seconds 2
            Invoke-Shot "muted" {
                param($s)
                Check "muted: right display shows the mute icon instead of a volume" ($s.Lit.right_center -gt 150 -and -not (Test-Digits $s "$mid")) (Describe-Shot $s)
                Check "muted: left display still shows the input name" (Test-LeftShows $s $range.name) (Describe-Shot $s)
            }
            Send-Remote @{ mute = "toggle" } | Out-Null
            Start-Sleep -Seconds 2
            Invoke-Shot "unmuted" { param($s) Check "unmuted: right display shows volume $mid again" (Test-Digits $s "$mid") (Describe-Shot $s) }

            Invoke-WebRequest "$base/MUTE" -UseBasicParsing | Out-Null
            Start-Sleep -Seconds 2
            Invoke-Shot "mute-url" { param($s) Check "GET /MUTE: right display shows the mute icon" ($s.Lit.right_center -gt 150 -and -not (Test-Digits $s "$mid")) (Describe-Shot $s) }
            Invoke-WebRequest "$base/UNMUTE" -UseBasicParsing | Out-Null
            Start-Sleep -Seconds 2
            Invoke-Shot "unmute-url" { param($s) Check "GET /UNMUTE: right display shows volume $mid again" (Test-Digits $s "$mid") (Describe-Shot $s) }
        }

        Write-Host "-- WiFi signal and temperature indicators"
        Wake-Display
        Start-Sleep -Seconds 2
        Invoke-Shot "indicators-on" {
            param($s)
            Check "WiFi signal indicator shown" ($s.Lit.right_wifi -gt 15) (Describe-Shot $s)
            Check "temperature indicators shown (DisplayTemperature1/2 = 3)" ($s.Lit.right_temp -gt 40) (Describe-Shot $s)
        }
        Set-DeviceSettings @{ DisplayTemperature1 = 0; DisplayTemperature2 = 0 }
        Wake-Display
        Start-Sleep -Seconds 2
        Invoke-Shot "temperature-off" { param($s) Check "temperature indicators hidden (DisplayTemperature1/2 = 0)" ($s.Lit.right_temp -lt 10) (Describe-Shot $s) }
        Set-DeviceSettings @{ DisplayTemperature1 = 3; DisplayTemperature2 = 3 }

        Write-Host "-- Screen saver"
        Set-DeviceSettings @{ ScreenSaverActive = $true; DisplayTimeout = 3 }
        Wake-Display
        Start-Sleep -Seconds 5
        Invoke-Shot "screensaver-on" { param($s) Check "screen saver blanks both displays after the timeout" ($s.Lit.left -lt 20 -and $s.Lit.right -lt 20) (Describe-Shot $s) }
        # A longer timeout (a settings POST is not user activity) keeps the wake capture from racing the saver
        Set-DeviceSettings @{ DisplayTimeout = 20 }
        Wake-Display
        Start-Sleep -Seconds 2
        $current = Get-State
        $currentName = $current.inputs[$current.selectedInput].name
        Invoke-Shot "screensaver-wake" {
            param($s)
            Check "remote activity wakes the displays" ($s.Lit.left -gt 100 -and $s.Lit.right -gt 100) (Describe-Shot $s)
            Check "left display shows the input name after waking" (Test-LeftShows $s $currentName) (Describe-Shot $s)
        }
        Set-DeviceSettings @{ ScreenSaverActive = $false }
        Wake-Display
        Start-Sleep -Seconds 6
        Invoke-Shot "screensaver-off" { param($s) Check "displays stay on when the screen saver is disabled" ($s.Lit.left -gt 100 -and $s.Lit.right -gt 100) (Describe-Shot $s) }
        Set-DeviceSettings @{ ScreenSaverActive = $true; DisplayTimeout = 255 }

        Write-Host "-- Input name hidden (DisplaySelectedInput = off)"
        $state = Get-State
        $other = @(0..4 | Where-Object { $state.inputs[$_].active -and $_ -ne $state.selectedInput })[0]
        if ($null -eq $other) {
            Write-Host "SKIP  Only one active input." -ForegroundColor Yellow
        }
        else {
            Set-DeviceSettings @{ DisplaySelectedInput = $false }
            Send-Remote @{ input = $other } | Out-Null
            Start-Sleep -Seconds 3
            Invoke-Shot "name-hidden" { param($s) Check "left display shows no input name when DisplaySelectedInput is off" ($s.Lit.left -lt 20) (Describe-Shot $s) }
            Set-DeviceSettings @{ DisplaySelectedInput = $true }
            Send-Remote @{ input = $state.selectedInput } | Out-Null
            Start-Sleep -Seconds 3
            Invoke-Shot "name-restored" { param($s) Check "left display shows the input name again" (Test-LeftShows $s $state.inputs[$state.selectedInput].name) (Describe-Shot $s) }
        }
    }
    catch {
        Check "webcam display test" $false $_.Exception.Message
    }
    finally {
        try { Set-DeviceSettings $restore } catch { Write-Host "Could not restore display settings: $_" -ForegroundColor Yellow }
    }
}

# --- Checks that need the preamp to power off/on (used by the -IncludePower section) ---

function Test-DisplaysBlankInStandby {
    Invoke-Shot "standby" { param($s) Check "standby: both displays are off" ($s.Lit.left -lt 20 -and $s.Lit.right -lt 20) (Describe-Shot $s) }
}

# Lengthens the on-delay of the active triggers so the "Wait... N" countdown stays on screen long enough to capture.
# Returns the settings to restore, or $null when no trigger is active.
function Set-StartupCountdownDelays {
    $config = Invoke-RestMethod "$base/api/settings" -TimeoutSec 10
    $values = @{}
    $restore = @{}
    foreach ($n in 1, 2) {
        if ([int]$config.("Trigger${n}Active")) {
            $values["Trigger${n}OnDelay"] = 12
            $restore["Trigger${n}OnDelay"] = [int]$config.("Trigger${n}OnDelay")
        }
    }
    if ($values.Count -eq 0) { return $null }
    Set-DeviceSettings $values
    $restore
}

# Call right after the power-on request: the left display shows trigger 1 and the right display trigger 2
function Test-StartupCountdown($restore) {
    Start-Sleep -Seconds 2
    Invoke-Shot "countdown" {
        param($s)
        if ($restore.ContainsKey("Trigger1OnDelay")) {
            Check "start-up: left display shows the trigger 1 countdown" ((Normalize-DisplayText $s.Left).Contains("wait")) (Describe-Shot $s)
        }
        if ($restore.ContainsKey("Trigger2OnDelay")) {
            Check "start-up: right display shows the trigger 2 countdown" ((Normalize-DisplayText $s.Right).Contains("wait")) (Describe-Shot $s)
        }
    }
}

function Test-DisplaysAfterStartup {
    $state = Get-State
    $name = $state.inputs[$state.selectedInput].name
    Invoke-Shot "after-startup" {
        param($s)
        Check "after start-up: left display shows the input name" (Test-LeftShows $s $name) (Describe-Shot $s)
        Check "after start-up: right display is on" ($s.Lit.right -gt 100) (Describe-Shot $s)
    }
}

function Remove-DisplayCaptures([bool]$keep) {
    if (-not (Test-Path $script:displayDir)) { return }
    if ($keep) { Write-Host "Display captures retained for diagnosis: $script:displayDir" -ForegroundColor Yellow }
    else { Remove-Item $script:displayDir -Recurse -Force -ErrorAction SilentlyContinue }
}
