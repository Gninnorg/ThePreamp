param(
    [Parameter(Mandatory = $true)][string]$Target,
    [switch]$IncludePower,
    [switch]$IncludeDisplay,
    [int]$CameraIndex = -1,
    [string]$TesseractPath,
    [string]$MqttHost,
    [string]$BaseTopic = "thepreamp"
)

# Black-box test of the web API (and optionally MQTT) of a running preamp.
# Restores the original input, volume and mute state afterwards.

$base = "http://$Target"
$script:pass = 0
$script:fail = 0

function Check([string]$name, [bool]$ok, [string]$detail = "") {
    if ($ok) { $script:pass++; Write-Host "PASS  $name" -ForegroundColor Green }
    else { $script:fail++; Write-Host "FAIL  $name $detail" -ForegroundColor Red }
}

function Get-State { Invoke-RestMethod "$base/api/remote" -TimeoutSec 10 }

function Send-Remote([hashtable]$body) {
    Invoke-RestMethod "$base/api/remote" -Method Post -Body $body -TimeoutSec 10
}

function Expect-Http([string]$name, [scriptblock]$call, [int]$status) {
    try { & $call | Out-Null; $code = 200 }
    catch { $code = [int]$_.Exception.Response.StatusCode }
    Check $name ($code -eq $status) "(got HTTP $code, expected $status)"
}

try { $orig = Get-State } catch { Write-Host "Cannot reach $base : $_" -ForegroundColor Red; exit 2 }
if ($orig.standby) { Write-Host "Preamp is in standby - turn it on first." -ForegroundColor Yellow; exit 2 }
Check "remote state includes temperature array" ($null -ne $orig.temperatures -and $orig.temperatures -is [array])
if ($null -ne $orig.temperatures) {
    $valid = $true
    foreach ($sensor in $orig.temperatures) {
        if ($sensor.channel -notin 0,1,2,3 -or ($null -ne $sensor.celsius -and ($sensor.celsius -isnot [ValueType] -or $sensor.celsius -lt 0 -or $sensor.celsius -gt 99))) { $valid = $false }
    }
    Check "remote temperature readings have valid channels and values" $valid
}
if ($IncludeDisplay) {
    $script:pythonCommand = Get-Command python -ErrorAction SilentlyContinue
    if (-not $script:pythonCommand) { Write-Host "Python was not found on PATH." -ForegroundColor Red; exit 2 }
    if (-not $TesseractPath) {
        $tesseractCommand = Get-Command tesseract -ErrorAction SilentlyContinue
        if ($tesseractCommand) { $TesseractPath = $tesseractCommand.Source }
    }
    if (-not $TesseractPath -or -not (Test-Path $TesseractPath)) {
        Write-Host "Tesseract OCR was not found. Install it or pass -TesseractPath." -ForegroundColor Red
        exit 2
    }

    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
    $script:cameraScript = Join-Path $projectRoot ".github\skills\preamp-webcam-display-test\scripts\capture_display.py"
    if (-not (Test-Path $script:cameraScript)) {
        Write-Host "Webcam capture helper was not found: $script:cameraScript" -ForegroundColor Red
        exit 2
    }
    & $script:pythonCommand.Source -c "import cv2" 2>$null
    if ($LASTEXITCODE -ne 0) {
        Write-Host "OpenCV was not found. Install it with: python -m pip install -r .github\skills\preamp-webcam-display-test\requirements.txt" -ForegroundColor Red
        exit 2
    }
    $calibrateScript = Join-Path (Split-Path $script:cameraScript) "calibrate_camera.py"
    $cameraArgs = if ($CameraIndex -ge 0) { @("--camera-index", $CameraIndex) } else { @() }
    # Re-sending the current volume counts as activity: it wakes the screen saver so the displays are lit
    function Wake-PreampDisplay { Send-Remote @{ volume = (Get-State).volume } | Out-Null }
    Write-Host "Checking webcam calibration..."
    Wake-PreampDisplay
    & $script:pythonCommand.Source $calibrateScript --check @cameraArgs
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Calibration is missing or no longer matches the camera view - starting calibration. Mark both displays, press 's' to save and 'q' to continue." -ForegroundColor Yellow
        # Keep the displays awake while the calibration window is open
        $keepAwake = Start-Job -ArgumentList $base -ScriptBlock {
            param($url)
            while ($true) {
                try {
                    $v = (Invoke-RestMethod "$url/api/remote" -TimeoutSec 5).volume
                    Invoke-RestMethod "$url/api/remote" -Method Post -Body @{ volume = $v } -TimeoutSec 5 | Out-Null
                } catch { }
                Start-Sleep -Seconds 3
            }
        }
        try { & $script:pythonCommand.Source $calibrateScript @cameraArgs }
        finally { Stop-Job $keepAwake; Remove-Job $keepAwake }
        Wake-PreampDisplay
        & $script:pythonCommand.Source $calibrateScript --check @cameraArgs
        if ($LASTEXITCODE -ne 0) { Write-Host "Calibration is still not valid - aborting." -ForegroundColor Red; exit 2 }
    }
    . (Join-Path $PSScriptRoot "display-tests.ps1")
}
Write-Host "Start state: input=$($orig.selectedInput) volume=$($orig.volume) muted=$($orig.muted)"

Write-Host "`n== Pages and static files =="
foreach ($p in "/", "/remote", "/settings.html", "/wifi.html", "/ir.html", "/update.html", "/buildinfo.txt") {
    try { $r = Invoke-WebRequest "$base$p" -UseBasicParsing -TimeoutSec 10; Check "GET $p" ($r.StatusCode -eq 200) }
    catch { Check "GET $p" $false $_.Exception.Message }
}
Expect-Http "missing stylesheet -> 404 (not 500)" { Invoke-WebRequest "$base/style.css" -UseBasicParsing -TimeoutSec 10 } 404

Write-Host "`n== Mute =="
if ((Get-State).muted) { Send-Remote @{ mute = "toggle" } | Out-Null }
Check "mute toggle -> muted" ((Send-Remote @{ mute = "toggle" }).muted -eq $true)
Check "mute toggle -> unmuted" ((Send-Remote @{ mute = "toggle" }).muted -eq $false)
Invoke-WebRequest "$base/MUTE" -UseBasicParsing | Out-Null
Check "GET /MUTE -> muted" ((Get-State).muted -eq $true)
Invoke-WebRequest "$base/UNMUTE" -UseBasicParsing | Out-Null
Check "GET /UNMUTE -> unmuted" ((Get-State).muted -eq $false)
Expect-Http "mute with invalid value -> 400" { Send-Remote @{ mute = "bogus" } } 400

Write-Host "`n== Volume (selected input) =="
$s = Get-State
$cur = $s.inputs[$s.selectedInput]
$min = [int]$cur.minVolume
$max = [int]$cur.maxVolume
foreach ($v in $min, [int](($min + $max) / 2), $max) {
    $r = Send-Remote @{ volume = $v }
    Check "set volume $v" ($r.volume -eq $v) "(got $($r.volume))"
}
Expect-Http "volume above max -> 400" { Send-Remote @{ volume = ($max + 1) } } 400
Expect-Http "volume below min -> 400" { Send-Remote @{ volume = ($min - 1) } } 400

Write-Host "`n== Inputs =="
for ($i = 0; $i -lt 5; $i++) {
    if ($s.inputs[$i].active) {
        $r = Send-Remote @{ input = $i }
        Check "select input $i ($($s.inputs[$i].name))" ($r.selectedInput -eq $i)
    }
    else {
        Expect-Http "inactive input $i rejected -> 400" { Send-Remote @{ input = $i } } 400
    }
}
Expect-Http "input 9 -> 400" { Send-Remote @{ input = 9 } } 400
Expect-Http "power with invalid value -> 400" { Send-Remote @{ power = "bogus" } } 400

Write-Host "`n== Settings and MQTT API =="
try {
    $settings = Invoke-RestMethod "$base/api/settings" -TimeoutSec 10
    Check "GET /api/settings returns VolumeSteps" ($null -ne $settings.VolumeSteps)
    Check "GET /api/settings does not return the WiFi password" ([string]::IsNullOrEmpty($settings.pass))
    Check "GET /api/settings includes all temperature settings" ($null -ne $settings.TemperatureEnabled0 -and $null -ne $settings.TemperatureEnabled1 -and $null -ne $settings.TemperatureEnabled2 -and $null -ne $settings.TemperatureEnabled3 -and $null -ne $settings.TemperatureSlot1 -and $null -ne $settings.TemperatureSlot2)
}
catch { Check "GET /api/settings" $false $_.Exception.Message }
if ($settings -and $null -ne $settings.TemperatureSlot1) {
    $temperatureRestore = @{
        TemperatureEnabled0 = [bool]$settings.TemperatureEnabled0
        TemperatureEnabled1 = [bool]$settings.TemperatureEnabled1
        TemperatureEnabled2 = [bool]$settings.TemperatureEnabled2
        TemperatureEnabled3 = [bool]$settings.TemperatureEnabled3
        TemperatureSlot1 = [int]$settings.TemperatureSlot1
        TemperatureSlot2 = [int]$settings.TemperatureSlot2
    }
    try {
        $temperatureTest = @{ TemperatureEnabled0 = $false; TemperatureEnabled1 = $true; TemperatureEnabled2 = $true; TemperatureEnabled3 = $false; TemperatureSlot1 = 2; TemperatureSlot2 = 3 }
        Invoke-RestMethod "$base/api/settings" -Method Post -Body @{ settings = ($temperatureTest | ConvertTo-Json -Compress) } -TimeoutSec 10 | Out-Null
        $actual = Invoke-RestMethod "$base/api/settings" -TimeoutSec 10
        Check "temperature settings round-trip" (-not $actual.TemperatureEnabled0 -and $actual.TemperatureEnabled1 -and $actual.TemperatureEnabled2 -and -not $actual.TemperatureEnabled3 -and $actual.TemperatureSlot1 -eq 2 -and $actual.TemperatureSlot2 -eq 3)
        $channels = @((Get-State).temperatures | ForEach-Object { [int]$_.channel })
        Check "remote exposes only enabled temperatures" ($channels.Count -eq 2 -and $channels[0] -eq 1 -and $channels[1] -eq 2)
        Expect-Http "duplicate temperature slots -> 400" { Invoke-RestMethod "$base/api/settings" -Method Post -Body @{ settings = '{"TemperatureSlot1":2,"TemperatureSlot2":2}' } -TimeoutSec 10 } 400
        Expect-Http "invalid temperature channel -> 400" { Invoke-RestMethod "$base/api/settings" -Method Post -Body @{ settings = '{"TemperatureSlot1":5}' } -TimeoutSec 10 } 400
    }
    catch { Check "temperature settings API" $false $_.Exception.Message }
    finally {
        try { Invoke-RestMethod "$base/api/settings" -Method Post -Body @{ settings = ($temperatureRestore | ConvertTo-Json -Compress) } -TimeoutSec 10 | Out-Null }
        catch { Check "restore temperature settings" $false $_.Exception.Message }
    }
}
try {
    $m = Invoke-RestMethod "$base/api/mqtt" -TimeoutSec 10
    Check "GET /api/mqtt has expected fields" ($null -ne $m.enabled -and $null -ne $m.baseTopic)
    Check "GET /api/mqtt does not return the password" ($null -eq $m.pass)
}
catch { Check "GET /api/mqtt" $false $_.Exception.Message }

if ($IncludeDisplay) { Invoke-DisplayTests }
if ($MqttHost) {
    Write-Host "`n== MQTT (needs mosquitto_pub / mosquitto_sub) =="
    if (-not (Get-Command mosquitto_pub -ErrorAction SilentlyContinue)) { Write-Host "SKIP  mosquitto clients not found" -ForegroundColor Yellow }
    else {
        function Mqtt-State {
            $o = & mosquitto_sub -h $MqttHost -t "$BaseTopic/state" -C 1 -W 5 2>$null
            if ($o) { $o | ConvertFrom-Json } else { $null }
        }
        $mqttState = Mqtt-State
        $remoteState = Get-State
        $temperatureKeys = @("temperature_external_right", "temperature_external_left", "temperature_internal_1", "temperature_internal_2")
        $expectedChannels = @($remoteState.temperatures | ForEach-Object { [int]$_.channel })
        $validTemperatures = $null -ne $mqttState
        for ($i = 0; $i -lt $temperatureKeys.Count; $i++) {
            $property = $mqttState.PSObject.Properties[$temperatureKeys[$i]]
            if ($null -eq $property) { $validTemperatures = $false; continue }
            if ($i -notin $expectedChannels -and $null -ne $property.Value) { $validTemperatures = $false }
            if ($null -ne $property.Value -and ($property.Value -isnot [ValueType] -or $property.Value -lt 0 -or $property.Value -gt 99)) { $validTemperatures = $false }
        }
        Check "MQTT state includes valid temperature fields" $validTemperatures
        & mosquitto_pub -h $MqttHost -t "$BaseTopic/mute/set" -m "ON"; Start-Sleep -Seconds 1
        Check "MQTT mute ON" ((Mqtt-State).mute -eq "ON" -and (Get-State).muted -eq $true)
        & mosquitto_pub -h $MqttHost -t "$BaseTopic/mute/set" -m "OFF"; Start-Sleep -Seconds 1
        Check "MQTT mute OFF" ((Mqtt-State).mute -eq "OFF" -and (Get-State).muted -eq $false)
    }
}

Write-Host "`n== Restoring start state =="
try {
    Send-Remote @{ input = $orig.selectedInput } | Out-Null
    Send-Remote @{ volume = $orig.volume } | Out-Null
    if ((Get-State).muted -ne $orig.muted) { Send-Remote @{ mute = "toggle" } | Out-Null }
}
catch { Write-Host "Could not fully restore state: $_" -ForegroundColor Yellow }

if ($IncludePower) {
    Write-Host "`n== Power (last, as it restarts the signal path) =="
    $beforeStandby = Get-State
    Send-Remote @{ power = "toggle" } | Out-Null
    Start-Sleep -Seconds 6
    $standbyState = Get-State
    Check "power toggle -> standby" ($standbyState.standby -eq $true)
    $cachedAvailable = $true
    foreach ($sensor in $beforeStandby.temperatures) {
        $matches = @($standbyState.temperatures | Where-Object { $_.channel -eq $sensor.channel })
        if ($matches.Count -ne 1 -or ($null -ne $sensor.celsius -and $null -eq $matches[0].celsius)) { $cachedAvailable = $false }
    }
    Check "standby preserves available temperature readings" $cachedAvailable

    $countdownRestore = $null
    if ($IncludeDisplay) {
        Test-DisplaysBlankInStandby
        try { $countdownRestore = Set-StartupCountdownDelays } catch { Check "set start-up trigger delays" $false $_.Exception.Message }
        if ($null -eq $countdownRestore) { Write-Host "SKIP  No active trigger: start-up countdown not checked." -ForegroundColor Yellow }
    }

    Send-Remote @{ power = "toggle" } | Out-Null
    if ($countdownRestore) { Test-StartupCountdown $countdownRestore }
    # Start-up takes as long as the longest trigger delay; wait for it to finish
    $deadline = (Get-Date).AddSeconds(45)
    do { Start-Sleep -Seconds 2; $standby = (Get-State).standby } while ($standby -and (Get-Date) -lt $deadline)
    Start-Sleep -Seconds 3
    Check "power toggle back -> on" ((Get-State).standby -eq $false)

    if ($countdownRestore) {
        try { Set-DeviceSettings $countdownRestore } catch { Write-Host "Could not restore trigger delays: $_" -ForegroundColor Yellow }
    }
    if ($IncludeDisplay) { Test-DisplaysAfterStartup }
}

if ($IncludeDisplay) { Remove-DisplayCaptures ($script:fail -gt 0) }

Write-Host "`nResult: $script:pass passed, $script:fail failed"
exit ([int]($script:fail -gt 0))
