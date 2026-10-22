param(
    [Parameter(Mandatory = $true)][string]$Target,
    [string]$EnvName = "esp32doit-devkit-v1"
)

$ErrorActionPreference = "Stop"
$project = Resolve-Path (Join-Path $PSScriptRoot "..\..\..\Software\ThePreamp Controller")
Set-Location $project

$vscodePio = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\pio.exe"
if (Get-Command pio -ErrorAction SilentlyContinue) { $pio = @("pio") }
elseif (Test-Path $vscodePio) { $pio = @($vscodePio); $env:PATH = "$(Split-Path $vscodePio);$env:PATH" }
else { $pio = @("python", "-m", "platformio") }
function Pio {
    $cmd = $pio[0]; $rest = @()
    if ($pio.Count -gt 1) { $rest += $pio[1..($pio.Count - 1)] }
    & $cmd @rest @args
    if ($LASTEXITCODE -ne 0) { throw "PlatformIO failed: $args" }
}

Write-Host "== Building firmware =="
Pio run -e $EnvName
Write-Host "== Building filesystem image =="
Pio run -e $EnvName -t buildfs

if ($Target -eq "usb") {
    Write-Host "== Uploading filesystem (USB) =="
    Pio run -e $EnvName -t uploadfs
    Write-Host "== Uploading firmware (USB) =="
    Pio run -e $EnvName -t upload
    return
}

$build = Join-Path $project ".pio\build\$EnvName"
function OtaUpload($mode, $file) {
    Write-Host "== OTA upload ($mode): $file =="
    $base = "http://$Target"
    $md5 = (Get-FileHash $file -Algorithm MD5).Hash.ToLower()
    curl.exe -sS -f "$base/ota/start?mode=$mode&hash=$md5" | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "OTA start failed ($mode)" }
    curl.exe -sS -f -F "file=@$file" "$base/ota/upload"
    if ($LASTEXITCODE -ne 0) { throw "OTA upload failed ($mode)" }
    Write-Host ""
}

$fs = Join-Path $build "spiffs.bin"
if (-not (Test-Path $fs)) { $fs = Join-Path $build "littlefs.bin" }
OtaUpload "fs" $fs
Start-Sleep -Seconds 3
OtaUpload "fr" (Join-Path $build "firmware.bin")
Write-Host "Done. The preamp will reboot."
