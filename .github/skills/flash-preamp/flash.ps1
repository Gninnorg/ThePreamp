param(
    [Parameter(Mandatory = $true)][string]$Target,
    [string]$EnvName = "esp32doit-devkit-v1",
    [switch]$Force
)

$ErrorActionPreference = "Stop"
$project = Resolve-Path (Join-Path $PSScriptRoot "..\..\..\Software\ThePreamp Controller")
Set-Location $project

$vscodePio = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\pio.exe"
if (Get-Command pio -ErrorAction SilentlyContinue) { $pio = @("pio") }
elseif (Test-Path $vscodePio) { $pio = @($vscodePio); $env:PATH = "$(Split-Path $vscodePio);$env:PATH" }
else { $pio = @("python", "-m", "platformio") }
function Invoke-Pio {
    $cmd = $pio[0]; $rest = @()
    if ($pio.Count -gt 1) { $rest += $pio[1..($pio.Count - 1)] }
    & $cmd @rest @args
    if ($LASTEXITCODE -ne 0) { throw "PlatformIO failed: $args" }
}

$build = Join-Path $project ".pio\build\$EnvName"
$stateDir = Join-Path $project ".pio\flash-state\$EnvName"
New-Item -ItemType Directory -Force $stateDir | Out-Null
$targetKey = $Target -replace '[^A-Za-z0-9.-]', '_'

# Hash of the inputs of an image. build_stamp.py rewrites build_info.h and data\buildinfo.txt on every
# run, so those two files are excluded or everything would always look changed.
function Get-InputHash($paths, $excludeNames) {
    $files = foreach ($p in $paths) {
        $full = Join-Path $project $p
        if (Test-Path $full -PathType Container) { Get-ChildItem $full -Recurse -File }
        elseif (Test-Path $full) { Get-Item $full }
    }
    $files = $files | Where-Object { $excludeNames -notcontains $_.Name } | Sort-Object FullName
    $lines = foreach ($f in $files) {
        $rel = $f.FullName.Substring($project.Path.Length)
        "$rel " + (Get-FileHash $f.FullName -Algorithm MD5).Hash
    }
    $bytes = [Text.Encoding]::UTF8.GetBytes(($lines -join "`n"))
    $md5 = [Security.Cryptography.MD5]::Create()
    ([BitConverter]::ToString($md5.ComputeHash($bytes))) -replace '-', ''
}
function Read-State($name) {
    $f = Join-Path $stateDir $name
    if (Test-Path $f) { (Get-Content $f -Raw).Trim() } else { "" }
}
function Write-State($name, $value) { Set-Content (Join-Path $stateDir $name) $value -NoNewline }

$fwHash = Get-InputHash @("src", "include", "lib", "platformio.ini", "build_stamp.py") @("build_info.h")
$fsHash = Get-InputHash @("data") @("buildinfo.txt")

$fsImage = Join-Path $build "spiffs.bin"
if (-not (Test-Path $fsImage)) { $fsImage = Join-Path $build "littlefs.bin" }
$fwImage = Join-Path $build "firmware.bin"

$needFwBuild = $Force -or (Read-State "fw.built") -ne $fwHash -or -not (Test-Path $fwImage)
$needFsBuild = $Force -or (Read-State "fs.built") -ne $fsHash -or -not (Test-Path $fsImage)

if ($needFwBuild) {
    Write-Host "== Building firmware (sources changed) =="
    Invoke-Pio run -e $EnvName
    Write-Host "== Firmware build done =="
    Write-State "fw.built" $fwHash
}
else { Write-Host "== Firmware unchanged, skipping build ==" }

if ($needFsBuild) {
    Write-Host "== Building filesystem image (data changed) =="
    Invoke-Pio run -e $EnvName -t buildfs
    Write-State "fs.built" $fsHash
}
else { Write-Host "== Filesystem unchanged, skipping build ==" }

# Re-check after building: the image on disk is what gets uploaded
$fsImage = Join-Path $build "spiffs.bin"
if (-not (Test-Path $fsImage)) { $fsImage = Join-Path $build "littlefs.bin" }

$needFwUpload = $Force -or $needFwBuild -or (Read-State "fw.$targetKey.uploaded") -ne $fwHash
$needFsUpload = $Force -or $needFsBuild -or (Read-State "fs.$targetKey.uploaded") -ne $fsHash

if (-not $needFwUpload -and -not $needFsUpload) {
    Write-Host "Nothing changed since the last upload to $Target. Use -Force to flash anyway."
    return
}

if ($Target -eq "usb") {
    # nobuild stops PlatformIO from rebuilding just because build_stamp.py touched build_info.h
    if ($needFsUpload) {
        Write-Host "== Uploading filesystem (USB) =="
        Invoke-Pio run -e $EnvName -t nobuild -t uploadfs
        Write-State "fs.$targetKey.uploaded" $fsHash
    }
    if ($needFwUpload) {
        Write-Host "== Uploading firmware (USB) =="
        Invoke-Pio run -e $EnvName -t nobuild -t upload
        Write-State "fw.$targetKey.uploaded" $fwHash
    }
    return
}

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

if ($needFsUpload) {
    OtaUpload "fs" $fsImage
    Write-State "fs.$targetKey.uploaded" $fsHash
    if ($needFwUpload) { Start-Sleep -Seconds 3 }
}
if ($needFwUpload) {
    OtaUpload "fr" $fwImage
    Write-State "fw.$targetKey.uploaded" $fwHash
    Write-Host "Done. The preamp will reboot."
}
else { Write-Host "Done. Only the filesystem was updated, firmware untouched." }
