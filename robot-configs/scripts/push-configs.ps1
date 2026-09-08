param(
    [string]$DeviceSerial
)

$ErrorActionPreference = "Stop"
$configDirectory = Split-Path -Parent $PSScriptRoot
$configs = @(Get-ChildItem -Path $configDirectory -Filter "*.xml" -File -Recurse)

if ($configs.Count -eq 0) {
    throw "No XML configuration files found in $configDirectory"
}

$adbDevices = @(adb devices | Select-String "`tdevice$")
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
    & adb -s $DeviceSerial push $config.FullName $destination
    if ($LASTEXITCODE -ne 0) {
        throw "ADB failed while pushing $($config.Name)"
    }
}

Write-Host "Pushed $($configs.Count) configuration file(s)."
Write-Host "Select the desired profile in Driver Station -> Configure Robot."
