# Robot hardware configurations

These XML files map FTC hardware names to hub ports. Keep the repository copy
in sync with the physical wiring, and verify every port before uploading.

## Sensor profiles

The full BioBuzz XML files have matching `biobuzz-starterbot.profile.json` and
`biobuzz-competitionbot.profile.json` files. The XML remains an ordinary FTC
hardware configuration; sensor mounting settings live only in its JSON companion.
`./gradlew pushRobotConfigs` uploads both XML files to `/sdcard/FIRST/` and JSON
profiles to `/sdcard/FIRST/robot-profiles/`. After uploading, select the matching
XML in Configure Robot. The active configuration's name must match the JSON
basename, including case; rename the pair together and re-upload if needed.

StarterBot, CompetitionBot, Configuration Test, and both OTOS tests look up the
active profile at INIT. StarterBot and CompetitionBot reject a profile for the
other robot; the field-navigation experiment is restricted to StarterBot's
18-inch geometry and measured raw OTOS axis convention. Missing, invalid,
or incompatible profiles stop initialization rather than defaulting.
Drive-only and teaching/sample OpModes have not been migrated and continue to
use their existing configuration conventions.

The StarterBot profile records the existing hub orientation (logo RIGHT, USB UP),
Limelight name and pipeline 0, and the approved OTOS measurements: optical
center 19 cm from the back and 7.5 cm from the left of the 18-inch square robot.
OTOS scalars of 1.0 are nominal, **not calibrated**. Field navigation leaves
the OTOS hardware offset at zero and subtracts the rotating lever arm in the
navigator exactly once. The direct-return OTOS test intentionally uses zero
hardware offset and raw sensor coordinates.

The CompetitionBot JSON currently copies StarterBot's sensor settings. These
are placeholders, not CompetitionBot measurements; update and upload the
profile when its mounting details are known. Inspect the active configuration
name shown on the configuration screen before driving.

The current vision algorithm assumes the camera faces the launcher, and the
field-navigation experiment assumes raw OTOS forward = negative X, right =
positive Y, and clockwise = negative heading. The profile enforces those
assumptions; other mounting orientations are not supported yet. Limelight
position/full rotation and general OTOS orientation transforms have not been
measured or implemented and must not be added as arbitrary JSON fields.
Validate IMU, tracking, and drive directions at low speed on the physical robot.

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
I2C bus 0, port 0. The paired sensor profile supplies hub orientation and
Limelight name and pipeline. The hardware configuration also includes an OTOS device.

The competition profile maps `left_launcher`, `right_launcher`, and `gate`.
The CompetitionBot TeleOp uses the shared launcher subsystem with the left motor
REVERSE and the right motor FORWARD. Both receive the same encoder ticks/second
target. Readiness requires every motor above target minus 50 ticks/s together for
200 ms; any motor at or below target minus 75 ticks/s removes readiness.
The StarterBot windmill is a separate feeder subsystem. Competition positional
gate actuation is deferred until safe blocked/feeding positions are measured;
the CompetitionBot TeleOp does not command `gate`.
The competition TeleOp now uses the same `TeleopController` and controls as the
StarterBot, including hive tracking, slow aiming translation, rumble and alliance
LEDs. Drive mode, launcher target and alliance currently share the StarterBot
preferences; the saved drive mode determines robot- versus field-centric driving.
Competition intake servos are disabled because its profile has only an intake motor.

## OTOS sensor test

Select **OTOS Sensor Test** in the Sensor Tests group with a profile containing
the SparkFun OTOS named `otos` (both full BioBuzz profiles include it).
This is the original direct-return test: INIT or **Options / Start** zeros tracking
and sets the current sensor position and heading as home, anywhere on the floor.
**A / cross** returns directly to that pose without a field map, robot dimensions,
mounting-offset compensation, wall docking, or obstacle avoidance. Keep the
entire return path clear.

The four named drive motors and `imu` are also required. Keep the robot flat
and still during INIT. Left stick translates, right stick X turns, and the
default power limit is 45%. D-pad up/down changes both manual and return power
by 5 percentage points per press (10%-100%). **Y / triangle stops the motors and
cancels return without changing home**, overriding A, Options and stick input.
A again, B, or stick input also cancels return. X calibrates the IMU only while stationary. Sensor warnings cancel a
return; errors/non-finite readings stop driving. Return times out at 12 seconds
and completes within 1 inch and 3 degrees after 0.2 seconds stationary.
STOP zeros drive power. Motors remain stopped before START.

Options / Start sets home once per press; these are SDK aliases. A rear paddle
mapped to Y now stops rather than resets home. Disable unwanted rear mappings
on the controller.

## OTOS field navigation test

Select **OTOS Field Navigation Test** in the Sensor Tests group for the separate
dimension-aware arena experiment. It uses the same configured hardware as the
original sensor test; the behavior below applies only to this field version.
The test also requires the four named drive motors and `imu` used by the shared
drive subsystem. Before INIT, place the 18 x 18 inch robot with its **back
against the red alliance wall, facing inward, center aligned with the red hive
pivot** (the midpoint along that wall). Keep it flat and completely still:
the test calibrates the OTOS IMU and zeros raw tracking. Motors remain stopped
before START. This placement establishes field coordinates; INIT somewhere else
invalidates the map. Do not lift or reposition the robot without reinitializing
at this same starting location.

Readings are displayed before and after START: X/Y position in inches, heading
in degrees, velocity, acceleration, and sensor warnings/errors. After START,
use the left stick for robot-relative translation and right stick X to turn,
with a default wheel power limit of 45%. D-pad up / down increases / decreases
the limit by 5 percentage points per press (10%-100%), for both manual driving
and return-to-home. The current limit is shown in telemetry and resets to 45%
when the OpMode is reinitialized. Translate a measured
distance and rotate it to confirm position and heading change; stationary zero
readings alone do not establish that optical tracking works. Heading wraps at
180 degrees. Raw readings retain the measured sensor axes: forward is negative
X, right is positive Y, clockwise decreases heading. Separately, the navigator
compensates the sensor optical-center position measured 19 cm from the back and
7.5 cm from the left: 3.86 cm behind and 15.36 cm left of robot center. Raw
sensor position and velocity are transformed to robot-center field coordinates,
including the rotating offset. Unit scalars are still uncalibrated.

Field coordinates are inches from arena center: +X toward the blue alliance
wall and +Y away from the audience. The initial robot center is (-63, 0), facing
+X. Field pose, distance to home and remaining waypoints appear in telemetry.

Press **A / cross** to plan a return to the initial robot-center position and
heading, or the most recent saved home. Routes go **around the entire hive frame,
never under it or over its bars**. The map uses the official 144-inch field and
49.46 x 38.95 inch centered frame from the
[FIRST competition manual](https://ftc-resources.firstinspires.org/ftc/game/manual),
section 9.6.1 / figure 9-8, pp. 69-70 (TU08), with placement from the
[field setup guide](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/eventfieldguide),
section 9.2. The simulator's older approximated collision bars are not used.

The planner expands the frame by the 18-inch square robot's half-diagonal
(12.73 inches) plus 2 inches of clearance so rotation is also covered. It
reserves a conservative 6-inch strip along all walls for flower structures,
plus the same robot/clearance allowance. This deliberately sacrifices some
usable space rather than assuming exact flower locations. The sole exception
is a narrow, straight approach to the red-wall starting bay at field Y = 0.
The robot aligns in clear space before entering that lane, does not turn/strafe
there, limits approach command to 25% before the selected power multiplier,
and targets 1 inch short of the wall (with a 0.5-inch position tolerance).

Other waypoints require a stop within 0.5 inch; final heading tolerance is
3 degrees, held stationary for 0.2 seconds before declaring home.
Press A again, B / circle, or move either drive stick to cancel; stick input
immediately resumes manual driving. OTOS tracking/tilt warnings cancel a return;
sensor errors or non-finite readings stop all driving. Route deviation, inadequate
clearance or docking misalignment also stop a return. A return times out after
60 seconds, or 4 seconds without progress. If starting outside mapped clear
space, it refuses the return rather than guessing an escape route: manually
move inward/away from obstacles and retry. Watch drive-status telemetry.

**Manual driving is unchanged and is not obstacle-limited.** No camera, distance
sensor or other-robot avoidance is implemented. A static map cannot correct
OTOS drift or detect a misplaced structure. First test short moves at low power,
then verify frame clearance and docking on the real arena with B ready; the map
and controller are not a physical collision guarantee.

Y / triangle stops the motors and cancels return without changing home, with
priority over A, Options and sticks. Options / Start stops the motors, cancels
any return, and saves the current robot-center
position and heading as home **without resetting OTOS or field coordinates**.
It refuses unsafe home locations, preserving the previous home. In the red-wall
starting bay, it preserves the straight, heading-zero approach and wall standoff.
To re-establish field localization, return to the starting placement and INIT
again; do not use Options / Start to compensate for a lost field position.
Press X / square to stop and recalibrate the IMU
only while flat and stationary; calibration is refused if the sensor reports
motion. Release the sticks and wait for the robot to stop before pressing X.
Each action runs once per button press. STOP always zeros drive power.

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

```sh
./gradlew pushRobotConfigs
```

On Windows, use `gradlew.bat pushRobotConfigs`. This task works on Linux,
macOS, and Windows using ADB from the Android SDK configured for TeamCode
(including `sdk.dir` in `local.properties`); PowerShell and ADB on PATH are
not required.

The task uploads every XML file under `robot-configs/`, including lesson
profiles, to `/sdcard/FIRST/`, creating that directory if needed. Filenames
must be unique across subdirectories because uploads use a flat destination.
If multiple ADB devices are connected, specify the target:

```sh
./gradlew pushRobotConfigs -ProbotConfigDeviceSerial=<adb-device-serial>
```

Alternatively, set `ANDROID_SERIAL` in the environment or
`robotConfigDeviceSerial=<adb-device-serial>` in your local
`~/.gradle/gradle.properties` (also used by Android Studio).
The Gradle property takes precedence over `ANDROID_SERIAL`. A disconnected
or unauthorized target, or any failed ADB command, fails the task instead
of reporting a successful upload.

The standalone Windows PowerShell script is also available:

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
select the profile for the connected robot. The shared TeamCode Android Studio
run configuration uploads both the XML files and JSON sensor profiles before
deploying the Robot Controller APK via `pushRobotConfigs`. A separate local
TeamCode run configuration may not have this before-launch task; use the shared
configuration or add the Gradle task `pushRobotConfigs` to it. With multiple
devices connected, set the target as described above; the upload task does not
inherit Android Studio's device picker.
