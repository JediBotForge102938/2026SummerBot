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

Every profile-dependent operational/test OpMode looks up the active hardware
configuration at INIT and displays the selected name using **Robot profile**:

| OpModes | Profile compatibility |
|---------|-----------------------|
| BioBuzz StarterBot TeleOp | StarterBot only |
| BioBuzz CompetitionBot TeleOp | CompetitionBot only |
| Configuration Test, OTOS Sensor Test | Either robot |
| Landmark Travel Test | Either robot |
| Both Limelight cluster tests | Either robot; camera name/pipeline from profile |
| Launcher Tuning | Either robot; launcher motors/directions/PIDF from profile |

Robot-specific modes validate the `robot` field **after loading the active
configuration's profile**; they do not select a fixed StarterBot/CompetitionBot
filename. Missing, invalid, or incompatible profiles stop initialization rather
than silently selecting another robot. The supplied profiles are
`biobuzz-starterbot` and `biobuzz-competitionbot`; selecting an XML with a
different name requires an identically named JSON companion.

Teaching lessons and the original goBILDA sample intentionally remain standalone
examples using their lesson/sample wiring and hard-coded assumptions, not
profile-aware operational modes. In particular, the IMU lesson assumes logo
RIGHT/USB UP; use the profile-aware tests for robot-specific sensor behavior.

Both `pushRobotConfigs` and the Windows `push-configs.ps1` script upload paired
JSON profiles as well as XML files. If INIT reports a missing profile, upload
the pair to that Control Hub and verify the active name in Configure Robot.
Reinstalling the APK alone does not install JSON profiles.

The StarterBot profile records the existing hub orientation (logo RIGHT, USB UP),
Limelight name and pipeline 0, and the approved OTOS measurements: optical
center 18.8 cm from the back and 7.5 cm from the left of the 18-inch square robot.
OTOS scalars of 1.0 are nominal, **not calibrated**. Field navigation leaves
the OTOS hardware offset at zero and subtracts the rotating lever arm in the
navigator exactly once. The direct-return OTOS test intentionally uses zero
hardware offset and raw sensor coordinates.

The CompetitionBot profile has its own mounting measurements. Its camera optical
axis points 37 degrees above horizontal, faces rearward, and is inverted.
Inspect the active configuration name shown on the configuration screen before driving.

The profiles also store chassis length and width, and the Limelight lens
position and orientation: 9.5 cm from the back, centered left-to-right,
31.8 cm above the floor, facing rearward and tilted 37 degrees upward with
zero roll for StarterBot. Yaw is measured counterclockwise from robot forward;
pitch is positive upward; roll is clockwise looking in the camera's forward
direction. The field-tracking test uses these values for a provisional diagnostic
position, but Limelight observations are not fused into the field pose.
Set `limelight.inverted` to `true` when the camera is mounted upside down
(rotated 180 degrees around its lens axis); use `false` for an upright camera.
The flag normalizes observations for hive classification and aiming. Keep
`rollDegrees` as the physical mounting roll (180 degrees for an inverted
camera); the field-tracking diagnostic avoids applying that rotation twice.
Profiles without the flag retain the upright behavior. Limelight's stream
orientation is display-only and must not be used as an observation correction.
Verify hive classification and aiming direction on the physical robot.
The diagnostics include a separate mount-compensated roll in a level frame
aligned with the camera heading (+X right, +Y down, +Z horizontally forward).
Each tag's rotation is composed with the camera elevation and residual lens roll
before averaging; this is not subtraction of Euler angles. Mounting yaw selects
the frame's horizontal heading and does not change its roll. The robot is assumed
level: this correction does not compensate chassis tilt or AprilTag pose ambiguity.
The inversion-corrected camera-space pose remains unchanged for aiming and the
approximate position diagnostic, so those consumers do not double-apply elevation.

Both cluster Limelight tests display mount-compensated roll and the active
profile/pitch. The tracking test displays camera-space and mount-compensated roll
for each posed tag; the adapter test displays the cluster's camera-space roll.
Classification requires ascending spatial tag IDs and a mount-compensated roll
below the profile's `scorableRollThresholdDegrees` (65 degrees in both profiles).
At or above the threshold, or with descending IDs, the cell is NOT_SCORABLE.
Missing or indeterminate pose, fewer than two posed tags, or mixed tag order gives
UNKNOWN_POSE. A confirmed target can be retained with one visible tag only while
its roll remains below the threshold; one tag cannot establish a new target.
This threshold is provisional: verify it across viewing angles and distances.
The [official Competition Manual](https://ftc-resources.firstinspires.org/ftc/game/manual?season=biobuzz)
Figure 9-10 (page 71) labels a 30-degree nominal hive tilt. Sections 9.6.2 and
10.5.1 describe upward-facing cells and stable states/damper contact, not a
numeric vision-roll threshold. The nominal tilt can inform expected orientations
once the tag-to-cell rotation/sign is verified; it is not itself the cutoff.
The SDK specifies a cluster origin relative to the tags, but does not identify
its physical location on the cell or supply the stable-state orientation.
Do not derive a scoring cutoff from those offsets alone.

The adapter test also displays **Opening normal up (verify sign)** and its
minimum/maximum across posed tags. This transforms tag +Z through the same mounting
rotation and assumes it faces outward from the cell bottom, making the opening
normal its negative. Values range from -1 (opening down) to +1 (opening up).
Check that a known upward-facing cell is positive and the same cell after tipping
is negative, on a level robot. A bottom plane 30 degrees from horizontal predicts
an upward-component magnitude near 0.866. The per-tag range reveals disagreement
that a mean could hide. These values are diagnostic only and do not drive the gate.
Neither roll nor opening-normal readings certify damper contact or a fully settled hive.
Each profile also specifies the intake motor direction: FORWARD for StarterBot
and REVERSE for CompetitionBot. TeleOp intake defaults are 100% and 75%,
respectively. StarterBot's `intake.leftServoDirection` is REVERSE and
`intake.rightServoDirection` is FORWARD; both TeleOp and Auto Shoot use these
profile directions. Profiles without these optional fields retain the original
left FORWARD/right REVERSE defaults. TeleOp intake assists are 50% and 35%,
respectively. D-pad left/right
adjusts the TeleOp intake speed in 5% increments; the last setting is stored
separately by robot. Assist is added to the manual power and the result clamped
to the motor range. Auto Shoot's intake powers are unchanged.
`drive.mirrorStrafe` is false for StarterBot
and true for CompetitionBot, reversing only the robot-relative lateral drive
component after any field-centric transform. `launcher.motors` lists each motor
name and its `FORWARD` or `REVERSE` direction; `launcher.pidf` supplies the
velocity controller's P, I, D and F coefficients for that robot. StarterBot uses
(40, 0, 0, 12.5); CompetitionBot uses the provisional values (40, 0, 0, 13.0).
`launcher.feedVelocityMarginTicksPerSecond` is the readiness margin below target:
125 for CompetitionBot and 50 for StarterBot.
CompetitionBot's `launcher.gate` specifies the servo name and open/closed
positions. Re-upload both version 9 profiles with the updated app; older profiles
are rejected.

### Launcher tuning

Select **Launcher Tuning** under **2 - (Tests)**. INIT detects the active profile
and applies its launcher PIDF without spinning. After START, gamepad 1 D-pad
up/down selects Target, P, I, D, or F; left/right adjusts the selected row once
per press. Target starts at 1200 ticks/s and changes by 25 within 0-6000.
P changes by 1, I by 0.01, and D/F by 0.1; hold LB for ten-times-finer
PIDF steps. Coefficients cannot go below zero.
Hold RB to spin and release to stop; RB held across START must be released
first. Changes apply live to every configured launcher motor. The competition
gate stays closed and no intake/feeder is commanded.

Telemetry shows actual/target, signed speed error, a separate +/-50-ticks/s
tracking indicator, readiness, and the profile's feed threshold.
The tracking indicator is OFF unless spinning is commanded, and WITHIN at an
absolute error of 50 ticks/s or less. Actual is the slowest motor for a multi-motor launcher. READY
uses the profile margin, not exact target tracking: 1140/1200 is inside the
observed CompetitionBot 125-ticks/s margin but outside StarterBot's 50.
The wider competition margin is a readiness workaround, not a tracking fix;
use the separate 50-ticks/s indicator when tuning.
All tuning changes are temporary. Copy the final P/I/D/F into the matching
`launcher.pidf` profile and run `./gradlew pushRobotConfigs` to retain them.
An undershoot can reflect tuning (including F or I), but also battery voltage,
motor/controller saturation, encoder configuration, or mechanical load; compare
at several targets on the physical robot rather than assuming PIDF is the cause.

The current vision algorithm assumes the camera faces the launcher. The OTOS
sensor test uses `otos.mountingRotationDegrees` to rotate raw sensor X/Y
counterclockwise into the StarterBot reference axes: forward = negative X and
right = positive Y. StarterBot uses 0 degrees; CompetitionBot uses -90 degrees,
which maps raw forward -Y to reference -X and raw right -X to reference +Y.
Heading remains relative to the starting orientation. Raw telemetry remains
unrotated. Verify both axes and heading before attempting autonomous motion;
the CompetitionBot angle is operator-selected and not yet physically verified.
Limelight mounting measurements do not transform Pedro's OTOS field pose.
Validate localization and drive directions on the physical robot before navigation.

## Field map and simulator

`FieldMap` uses field-center coordinates (+X toward blue, +Y away from the
audience), approximate hive centers, flower wall attachment locations and
garden/loading-zone bounds based on the
[field setup guide](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/eventfieldguide)
sections 6, 8 and 11. Hive center and flower attachment locations are rough,
not surveyed tag locations or safe approach poses; PARK is the loading zone.
Field and placement tolerances mean even the starting pose is an estimate.

The desktop simulator uses the bundled robot profiles. Limelight is a no-target
mock. Telemetry is plain text without Driver Station colors. Place and orient the
simulated robot at the selected starting pose before INIT; the menu sets Pedro's
assumed pose but does not reposition the simulator.
Both full BioBuzz models initially spawn at the red/audience shooting pose
(X -12, Y -63, heading 90 degrees for an 18-inch chassis), not the nearby tile
center at Y -60. Selecting another INIT start still requires manually matching
the simulator placement. An initial placement error shifts the estimated pose.
The simulator window is resizable. Its telemetry sidebar starts at 420 pixels
wide, fills the available height, wraps long lines, and grows when the window
is enlarged; the field retains its original dimensions.
The simulator ignores OTOS calibration scalars. Its Pedro OTOS mode models an
ideal centered sensor and supports setting position; the legacy diagnostic mode
does not support setting position. Its idealized sensors and field are not a
substitute for robot tests.
The **biobuzz-competitionbot** entry uses its own XML and placeholder sensor
profile. Its chassis is an approximation, its mechanisms have no detailed
visualization, and its Limelight is also a no-target mock. The drive-only and
exercise entries do not have BioBuzz sensor profiles.
The StarterBot simulator approximates its lateral response by swapping front/back
wheel slots on each side. CompetitionBot uses the simulator's normal wheel mapping
to approximate its observed opposite lateral response. This preserves forward and
turn responses; the mapping does not assert how either physical robot is wired.
The teaching configurations retain their original wheel mapping.

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

The competition profile maps `launcher` to Expansion Hub port 3 and uses
FORWARD for its motor direction. Readiness requires its encoder speed above
target minus 125 ticks/s for 200 ms; at or below target minus 150 ticks/s removes
readiness. Its positional servo is named `gate`, with open position 0.0 and
closed position 0.3. In TeleOp, B toggles the gate during INIT or after START,
independently of launcher speed; it closes during initialization and STOP and
retains its position across START. Auto Shoot opens it only when
its launcher readiness, alignment and stationary checks permit feeding.
The StarterBot windmill is a separate feeder subsystem.
The competition TeleOp now uses the same `TeleopController` and controls as the
StarterBot, including hive tracking, selected-speed aiming translation, rumble and alliance
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

## Pedro 3 tuning and landmark travel

See Pedro Pathing's [tuning guide](https://pedropathing.com/docs/pathing/tuning).
With the Robot Controller running and connected, open AutoTune at
`http://192.168.43.1:10158`. AutoTune starts its temporary tuning OpMode through
the Driver Station. For each robot, run the Mecanum Tuner and OTOS Tuner first.
Copy the generated `MecanumConfig` and `OTOSConfig` into that robot's
`starterDrivetrainConfig` / `starterOtosConfig` or `competitionDrivetrainConfig` /
`competitionOtosConfig` methods in `PedroConstants.java`, then rebuild and deploy.
Verify OTOS mounting orientation and the localization test before running
Foresight Tuner; its generated `ForesightConfig` goes into the matching
`starterForesightConfig` or `competitionForesightConfig` method. Rebuild and deploy
again. The initial motor directions and OTOS settings are placeholders, not
calibrated values; the physical Landmark Travel Test cannot initialize until
the matching Foresight configuration has been supplied.

Landmark Travel Test uses Pedro's native OTOS and mecanum drivetrain. During INIT,
select the robot's actual starting pose and place the robot at the displayed
`Place robot at` pose; the selection does not move it. Starting poses next to a
flower are shifted toward the field center when needed for the profile width.
For the 18-inch StarterBot, Blue Audience is at x=10.65 instead of 12 inches,
and Red Scoring is at x=-10.65 instead of -12. The corresponding CompetitionBot
starts shift slightly to approximately x=+/-11.97.
D-pad selects from the landmark list: four flowers, two gardens, two parking
areas, and four shooting starts. Options/Start saves the current pose as an
additional destination for that run. A starts or cancels a planned route,
B/Y stops, X calibrates the OTOS IMU when stationary, and the sticks override
navigation. Flower destinations face the flower from outside its modeled
envelope; they are robot-center poses, not the wall attachment coordinates.
The other landmark targets retain their previous offsets, except for the
adjusted starting poses above. Manual drive is limited to 50%; Pedro controls
autonomous drive power rather than the old per-phase power limits.

The planner routes around a rectangular central frame and four individual flower
envelopes, using the profile's chassis dimensions. The frame dimensions are from
the [competition manual](https://ftc-resources.firstinspires.org/ftc/game/manual?season=biobuzz)
section 9.6.1. The flower envelopes are conservative rectangles based on the
ring and mounting-bracket dimensions and location in the
[official field CAD](https://cad.onshape.com/documents/a355e772e3d24813de7852ee/w/f106353168f1f92100b81259/e/95d1e1e442b4138cccaf2d73):
6.70 inches along the wall and 6.57 inches inward, covering the space to the wall.
They are bounding shapes, not exact flower outlines. Transit paths use the
robot's circumradius plus the manual's one-inch field tolerance; straight
departures and approaches near walls use the oriented nominal footprint with
no added field or motion-error margin. Pedro's segment skipping is disabled,
but actual movement can still deviate from planned lines or cut corners.
**The routes are not a physical collision-safety guarantee.** Keep the route
clear, supervise the robot, and be ready to use Driver Station STOP.

OTOS tuning scalars correct systematic distance and rotation measurement errors;
Foresight tuning measures dynamics and determines controller settings. Neither
establishes a worst-case localization, path-tracking, or stopping-error bound.
After tuning, run repeated straight, turning, and stop tests at representative
speeds, directions, battery levels, and surfaces. Compare the actual robot
position and heading with independent field marks, including the path and final
stopped pose; use those measurements to decide additional clearance before
operating near obstacles. Telemetry's peak estimated path error compares Pedro's
estimated pose to its closest path point, so it cannot reveal OTOS drift or
replace independent measurements. This experiment does not yet add measured
motion-error allowances to routes.

Simulator configurations and sensor behavior are idealized and separate from
physical tuning; they do not establish a robot tune. The simulator uses the same
conservative frame and flower rectangles; the artwork remains illustrative.
Its shared Foresight values are adjusted for the simulator's near-instant motor
response, including a 20 in/s path velocity constraint. These values are
simulator-specific and must not be copied as physical robot tuning.
The bundled simulator Pedro jar comes from
`Beta8397/virtual_robot` commit `7105cfa53402`, and may differ from the Android
Pedro 3.0.2 artifacts.

## StarterBot TeleOp controls

These are the one-controller mappings. In either mode, after START, tap the driver's touchpad top/bottom
to increase/decrease drive speed, or right/left to increase/decrease bumper
speed, by 5% per tap within 0–100%. Release between taps; the center is
ignored. Adjustments last only for the current run. The controller and Driver
Station must support touch coordinates; the simulator does not support them.
In the INIT menu, either controller's D-pad can navigate and change settings.
**Controllers** selects
one or two controllers and is saved between runs (default: one). In two-controller
mode, gamepad 1 keeps the sticks, A, left bumper, touchpad, and drive-mode/heading controls;
gamepad 2 uses the trigger, right bumper, and D-pad mechanism controls below.
The live speeds and controller count appear in telemetry.
Shooter-ready rumble goes to the gunner; the driver rumbles continuously while
A is held and a scorable hive is locked.

| Input | Action |
| --- | --- |
| Left stick | Drive translation |
| Right stick X | Manual rotation |
| A, held | Aim rotation at a visible scorable hive; translation remains manual |
| Left bumper, held | Use the configured bumper speed |
| Right / left trigger | Intake forward / reverse |
| Right bumper, held | Run launcher; feed after it reaches speed |
| D-pad up / down | Adjust launcher target by 25 encoder ticks/second (0–6000) |
| D-pad left / right | Adjust intake speed in 5% steps |
| Share + Options / Back + Start | Toggle robot-centric / field-centric drive |
| Options / Start alone, released | Reset heading in field-centric mode |

Drive mode, launcher target, and tracking alliance are saved in preferences.

CompetitionBot TeleOp shares these controls except that B toggles its gate
open or closed independently of launcher readiness. In two-controller mode,
gamepad 2 controls the gate both during INIT and after START. The right bumper runs the
launcher; readiness still controls the intake feed assist. B held while INIT
begins must be released before toggling, and holding B across START does not
cause an additional toggle. A disconnect does not transfer controls; Driver
Station STOP is the safety backup.

Both TeleOps offer Drive speed and Bumper speed in the INIT menu. D-pad left/right
adjusts each from 0% to 100% in 5% steps; selections are saved on the Robot
Controller. Defaults are 60% and 100%. Bumper speed may be higher or lower than
Drive speed. While aiming with A, translation uses the selected drive or bumper
speed, while automatic rotation remains at full strength. Translation yields to
rotation when wheel power is saturated.
The launcher target defaults to 1250 encoder ticks/second. Feeding is enabled
only after the launcher holds above its target threshold for 200 ms; a zero
target disables launcher feeding. Start and Options are SDK aliases, so the
drive-mode toggle uses Share + Options on PlayStation or Back + Start on Xbox.

Simulator dropdown entries correspond to the four XML files in this repository.
They are explicitly implemented simulated robots, not a dynamically discovered
list of XML files; new configurations require a simulator implementation.
The S2-04 exercise demonstrates incorrect port assignments.

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
