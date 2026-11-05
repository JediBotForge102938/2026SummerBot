# Robot hardware configurations

These XML files map FTC hardware names to hub ports. Keep the repository copy
in sync with the physical wiring, and verify every port before uploading.

## Profiles

| File | Purpose |
| --- | --- |
| `biobuzz-starterbot.xml` | Full StarterBot profile used by the BioBuzz StarterBot TeleOp and simulator. |
| `biobuzz-competitionbot.xml` | Competition bot wiring profile. |
| `starterbot-drive-only.xml` | Four-drive-motor profile for the basic StarterBot TeleOps and simulator; not a full BioBuzz profile. |
| `lessons/s2_04_configuration_exercise.xml` | Deliberately miswired lesson exercise; do not select for normal driving. |

The drive-only profile maps ports as follows:

| Port | Position | Configuration name |
| ---: | --- | --- |
| 0 | Rear left | `left_back_drive` |
| 1 | Rear right | `right_back_drive` |
| 2 | Front right | `right_front_drive` |
| 3 | Front left | `left_front_drive` |

The full StarterBot TeleOp requires the four named drive motors, `imu`,
`intake`, `launcher`, `left_intake_servo`, `right_intake_servo`,
`windmillServo`, and a Limelight named `limelight`. The IMU is configured on
I2C bus 0, port 0; its expected hub orientation is logo right, USB up. Vision
uses Limelight pipeline 0. The profile also includes an OTOS device.

The competition profile maps `left_launcher`, `right_launcher`, and `gate`;
it is a hardware profile, not an implementation of the competition TeleOp.

## StarterBot TeleOp controls

| Input | Action |
| --- | --- |
| Left stick | Drive translation |
| Right stick X | Manual rotation |
| A, held | Aim rotation at a visible scorable hive; translation remains manual |
| Left bumper, held | Use 35% drive power |
| Right / left trigger | Intake forward / reverse |
| Right bumper, held | Run launcher; feed after it reaches speed |
| D-pad up / down | Adjust launcher target by 25 encoder ticks/second (0–6000) |
| D-pad left / right | Select RED / BLUE tracking alliance |
| Share + Options / Back + Start | Toggle robot-centric / field-centric drive |
| Options / Start alone, released | Reset heading in field-centric mode |

Drive mode, launcher target, and tracking alliance are saved in preferences.
The launcher target defaults to 1250 encoder ticks/second. Feeding is enabled
only after the launcher holds above its target threshold for 200 ms; a zero
target disables launcher feeding. Start and Options are SDK aliases, so the
drive-mode toggle uses Share + Options on PlayStation or Back + Start on Xbox.

The simulator packages `biobuzz-starterbot.xml` for its BioBuzz robot and
`starterbot-drive-only.xml` for its basic StarterBot. The S2-04 exercise
simulator reads the lesson configuration to demonstrate incorrect port
assignments.

## Uploading configurations

With the Control Hub connected over USB and authorized for ADB, run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\robot-configs\scripts\push-configs.ps1
```

The script uploads every XML file under `robot-configs\`, including lesson
profiles, to `/sdcard/FIRST/`. If multiple ADB devices are connected, specify
the target:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\robot-configs\scripts\push-configs.ps1 `
  -DeviceSerial <adb-device-serial>
```

Uploading copies files but does not remove old files from the Control Hub or
activate a profile. In Driver Station, refresh the configuration list and
select the profile for the connected robot. The TeamCode Android Studio run
configuration also uploads the XML files before deploying the Robot Controller
APK.
