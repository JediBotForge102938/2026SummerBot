# Robot hardware configuration

The single `starterbot-drive-only.xml` profile supports both StarterBot
TeleOps. It contains only the four drive motors; intake hardware is intentionally
omitted and is optional in both OpModes.

| Port | Physical position | Configuration name |
| ---: | --- | --- |
| 0 | Rear left | `left_back_drive` |
| 1 | Rear right | `right_back_drive` |
| 2 | Front left | `left_front_drive` |
| 3 | Front right | `right_front_drive` |

The XML uses the FTC hardware-configuration format and the embedded Control Hub
metadata from the current robot. The motor names are intentionally distinct
from the vestigial names in the previous `BioBuzz` configuration.

`StarterBotChassisTeleop` uses only the rear pair (`left_back_drive` and
`right_back_drive`) for two-motor tank drive. `StarterBotMecChassisTeleop` uses
the same configuration's four motors for mecanum drive.

## Managing configurations

The XML file in this directory is the repository copy of the robot hardware
profile. Edit or replace it only after verifying the physical hub ports and
device names.

With the Control Hub connected over USB and authorized for ADB, push the
configuration with:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\robot-configs\scripts\push-configs.ps1
```

The script uploads all `*.xml` files directly inside `robot-configs\` to
`/sdcard/FIRST/`. To target a specific ADB device when more than one is
connected:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\robot-configs\scripts\push-configs.ps1 `
  -DeviceSerial <adb-device-serial>
```

After uploading, use **Driver Station -> Configure Robot** to refresh the
configuration list and activate `starterbot-drive-only`. Pushing a file does
not automatically activate it. The shareable `TeamCode` Android Studio run
configuration runs this upload task before deploying the Robot Controller APK.

## Managing configurations

The XML files in this directory are the repository copies of the robot
hardware profiles. Edit or replace them only after verifying the physical hub
ports and device names. Keep each profile focused on one wiring layout.

The simulator consumes `starterbot-drive-only.xml` from this directory at build
time. Its named drive motors are therefore derived from the same XML that is
uploaded to the Control Hub. Wheel positions are inferred from the semantic
names (`left_front`, `right_front`, `left_back`, and `right_back`), so changing
REV hub ports in the XML does not require simulator code changes. The simulator
still supplies the physical model (mecanum geometry, motor behavior, and
virtual sensors) separately.

With the Control Hub connected over USB and authorized for ADB, push every
configuration with:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\robot-configs\scripts\push-configs.ps1
```

The script uploads all `*.xml` files directly inside `robot-configs\` to
`/sdcard/FIRST/`. To target a specific ADB device when more than one is
connected:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\robot-configs\scripts\push-configs.ps1 `
  -DeviceSerial <adb-device-serial>
```

After uploading, use **Driver Station -> Configure Robot** to refresh the
configuration list and activate the desired profile. Pushing a file does not
automatically activate it. Deploy Robot Controller code from Android Studio;
the configuration XMLs are separate from the APK.
