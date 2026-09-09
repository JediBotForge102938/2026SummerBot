param(
    [string]$DeviceSerial
)

$ErrorActionPreference = "Stop"
$configDirectory = Split-Path -Parent $PSScriptRoot
$configs = @(Get-ChildItem -Path $configDirectory -Filter "*.xml" -File -Recurse)

if ($configs.Count -eq 0) {
    throw "No XML configuration files found in $configDirectory"
}

$adbPath = $null
$adbCommand = Get-Command adb.exe -CommandType Application -ErrorAction SilentlyContinue
if ($null -ne $adbCommand) {
    $adbPath = $adbCommand.Source
}

$sdkCandidates = @($env:ANDROID_SDK_ROOT, $env:ANDROID_HOME)
$localProperties = Join-Path (Split-Path -Parent (Split-Path -Parent $PSScriptRoot)) "local.properties"
if (Test-Path $localProperties) {
    $sdkProperty = Select-String -Path $localProperties -Pattern "^sdk\.dir=" | Select-Object -First 1
    if ($null -ne $sdkProperty) {
        $sdkCandidates += ($sdkProperty.Line -replace "^sdk\.dir=", "").Replace("\:", ":").Replace("\\", "\")
    }
}

foreach ($sdkDirectory in $sdkCandidates) {
    if (-not [string]::IsNullOrWhiteSpace($sdkDirectory)) {
        $candidate = Join-Path $sdkDirectory "platform-tools\adb.exe"
        if (Test-Path $candidate) {
            $adbPath = (Resolve-Path $candidate).Path
            break
        }
    }
}

if ([string]::IsNullOrWhiteSpace($adbPath)) {
    throw "Could not find adb.exe. Add Android SDK platform-tools to PATH, set ANDROID_SDK_ROOT or ANDROID_HOME, or configure sdk.dir in local.properties."
}

$adbDevices = @(& $adbPath devices | Select-String "`tdevice$")
if ($adbDevices.Count -eq 0) {
    throw "No authorized ADB device found"
}

if ([string]::IsNullOrWhiteSpace($DeviceSerial)) {
    if ($adbDevices.Count -ne 1) {
        throw "Multiple ADB devices found; rerun with -DeviceSerial <serial>"
    }

    $DeviceSerial = ($adbDevices[0].Line -split "\s+")[0]
}

foreach ($config in $configs) {
    $destination = "/sdcard/FIRST/$($config.Name)"
    Write-Host "Pushing $($config.Name) to $destination"
    & $adbPath -s $DeviceSerial push $config.FullName $destination
    if ($LASTEXITCODE -ne 0) {
        throw "ADB failed while pushing $($config.Name)"
    }
}

Write-Host "Pushed $($configs.Count) configuration file(s)."
Write-Host "Select the desired profile in Driver Station -> Configure Robot."
