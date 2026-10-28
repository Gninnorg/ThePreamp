param(
    [Parameter(Mandatory = $true)][string]$Target,
    [switch]$IncludePower,
    [switch]$IncludeDisplay,
    [int]$CameraIndex = 0,
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
if ($IncludeDisplay) {
    if ($CameraIndex -lt 0) { Write-Host "CameraIndex must be zero or greater." -ForegroundColor Red; exit 2 }
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
}
catch { Check "GET /api/settings" $false $_.Exception.Message }
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
    Send-Remote @{ power = "toggle" } | Out-Null
    Start-Sleep -Seconds 6
    Check "power toggle -> standby" ((Get-State).standby -eq $true)

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
