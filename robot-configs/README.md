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
| OTOS Field Navigation Test | StarterBot only (fixed 18-inch navigator) |
| Configuration Test, OTOS Sensor Test | Either robot |
| Field Tracking Test, Landmark Travel Test | Either robot |
| Leave & Park, Simple Leave & Park, Auto Shoot | Either robot |
| All three Limelight tests | Either robot; camera name/pipeline from profile |

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

The CompetitionBot JSON currently copies StarterBot's sensor settings. These
are placeholders, not CompetitionBot measurements; update and upload the
profile when its mounting details are known. Inspect the active configuration
name shown on the configuration screen before driving.

The profiles also store chassis length and width, and the Limelight lens
position and orientation: 9.5 cm from the back, centered left-to-right,
31.8 cm above the floor, facing rearward and tilted 37 degrees upward with
zero roll for StarterBot. Yaw is measured counterclockwise from robot forward;
pitch is positive upward; roll is clockwise looking in the camera's forward
direction. The field-tracking test uses these values for a provisional diagnostic
position, but Limelight observations are not fused into the field pose.
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
velocity controller's P, I, D and F coefficients for that robot. Both profiles
initially retain the previous values (40, 0, 0, 12.5), which have not been tuned
for CompetitionBot's launcher. `launcher.feedVelocityMarginTicksPerSecond` is
the readiness margin below target: 100 for CompetitionBot and 50 for StarterBot.
CompetitionBot's `launcher.gate` specifies the servo name and open/closed
positions. Re-upload both version 9 profiles with the updated app; older profiles
are rejected.

The current vision algorithm assumes the camera faces the launcher, and the
field tracking uses `otos.mountingRotationDegrees` to rotate raw sensor X/Y
counterclockwise into the StarterBot reference axes: forward = negative X and
right = positive Y. StarterBot uses 0 degrees; CompetitionBot uses -90 degrees,
which maps raw forward -Y to reference -X and raw right -X to reference +Y.
Heading remains relative to the starting orientation. Raw telemetry remains
unrotated. Verify both axes and heading before attempting autonomous motion;
the CompetitionBot angle is operator-selected and not yet physically verified.
Limelight mounting measurements are stored but not used to transform observations.
Validate IMU, tracking, and drive directions at low speed on the physical robot.

## Field tracking test

Select **Field Tracking Test** in **2 - (Tests)**. Before INIT, place the
robot against the audience or scoring wall, centered in line with its alliance's
hive and with its front facing the hive. Keep the robot flat and still during
OTOS calibration. Select alliance, starting side and drive mode during INIT;
these establish an approximate robot-center starting pose at START. The test
permits manual stick driving at 50% power; hold Y to stop.

Telemetry reports the estimated field position in inches and hub-IMU heading in
degrees, raw OTOS heading, and each visible Limelight cell in a separate vision
section. For cells classified as scorable, it also shows an approximate vision
X/Y and the vision-minus-OTOS X/Y delta. The vision estimate uses the same IMU
heading, the camera mounting profile and the approximate 2D cell center; it is
not an independent heading estimate or a surveyed tipped-cell pose. Vision
readings may be delayed while OTOS readings are current, so compare while stopped.
The estimated pose combines OTOS displacement and hub-IMU heading with the
profile's OTOS lever arm. A sensor warning or invalid reading invalidates the
pose until reinitialization. Vision is observational only: scorable classification
does not establish that a cell is fully tipped and stable, so no vision correction
is applied. Test slowly and do not
use this experimental estimate for autonomous navigation or collision avoidance.

`FieldMap` uses field-center coordinates (+X toward blue, +Y away from the
audience), approximate hive centers, flower wall attachment locations and
garden/loading-zone bounds based on the
[field setup guide](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/eventfieldguide)
sections 6, 8 and 11. Hive center and flower attachment locations are rough,
not surveyed tag locations or safe approach poses; PARK is the loading zone.
Field and placement tolerances mean even the starting pose is an estimate.

The desktop simulator's **biobuzz-starterbot** uses the bundled
StarterBot JSON profile and simulates OTOS translation and its mounting offset.
Limelight is a no-target mock, so vision remains searching and cannot provide
vision position or delta. Telemetry is plain text without Driver Station colors.
Place and orient the simulated robot at the selected starting pose before INIT;
the menu sets the estimator's assumed pose but does not reposition the simulator.
Both full BioBuzz models initially spawn at the red/audience shooting pose
(X -12, Y -63, heading 90 degrees for an 18-inch chassis), not the nearby tile
center at Y -60. Selecting another INIT start still requires manually matching
the simulator placement. A three-inch initial placement error shifts every
tracked destination by three inches; it can make one garden look too close to
its wall and the other too far away despite both reporting arrival.
The simulator window is resizable. Its telemetry sidebar starts at 420 pixels
wide, fills the available height, wraps long lines, and grows when the window
is enlarged; the field retains its original dimensions.
The simulator ignores OTOS calibration scalars, supports only zero OTOS hardware
offset, and does not support setting the OTOS position. Its idealized sensors and
field are not a substitute for robot tests.
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
target minus 250 ticks/s for 200 ms; at or below target minus 275 ticks/s removes
readiness. Its positional servo is named `gate`, with open position 0.0 and
closed position 0.3. In TeleOp, B toggles the gate during INIT or after START,
independently of launcher speed; it closes during initialization and STOP and
retains its position across START. Auto Shoot opens it only when
its launcher readiness, alignment and stationary checks permit feeding.
The StarterBot windmill is a separate feeder subsystem.
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
compensates the sensor optical-center position measured 18.8 cm from the back and
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

## Leave & Park autonomous

Select **Leave & Park** in **1 - (Autonomous)**. During INIT, select the alliance
and audience/scoring starting spot using the shared configuration menu, and
place the robot at that mapped starting pose. START automatically begins one
Pedro route to the nearest end of the selected alliance's parking box, then stops:
Audience starts use the audience end; Scoring starts use the scoring end.
The robot backs into parking and finishes facing inward toward the open field,
away from the side wall, ready to drive forward out of parking.
Chassis overlap along the box's length starts at profile `parkingOverlapIn`
(2 inches with the supplied profiles). Red Audience uses that baseline;
Blue Audience and Blue Scoring add 1 inch (3 inches total); Red Scoring adds
2 inches (4 inches total), leaving the middle free for the alliance partner.
The planner and runtime clearance checks reserve the central half of the box
against the swept chassis footprint; routes do not drive through that space.
The existing center-parking rows in Landmark Travel Test are unchanged.
The drive-mode selection does not affect autonomous field-coordinate routing.
No A press, manual driving, mechanism actuation, or automatic restart occurs.
B/Y cancels; Driver Station STOP also stops the drive.

This uses the same profile power limits, tracking, clearance checks, arrival
criteria, and physical-readiness gate as Landmark Travel Test. Both supplied
physical profiles remain blocked until geometry and Pedro tuning are verified;
simulator experiments remain enabled. The parking target is an approach pose,
not a claim that the entire 18-inch chassis fits within the 11-inch parking strip.

## Simple Leave & Park

Select **Simple Leave & Park** for a straight-only alternative, on either
robot. There are no configuration selections or mapped parking coordinates.
INIT displays how far it will move: one chassis length from the active profile
(18 inches for StarterBot; 45 cm, about 17.7 inches, for CompetitionBot).
Face the desired parking target and place the robot one chassis length behind
its desired final position, measured using the same chassis point at both ends.
Keep the robot flat and still during INIT.

After START it drives straight forward at the profile's `finalApproachPower`
until corrected OTOS displacement reaches that chassis length.
Tracking uses a local starting frame, independent of alliance or field heading.
Verified profile geometry is required; Pedro tuning is not used.
All numeric motion limits come from `navigation`: `headingToleranceDegrees`
and `arrivalToleranceIn` bound heading and sideways drift; `timeoutSeconds` bounds total
travel time; `progressThresholdIn` and `progressTimeoutSeconds` detect stalls.
These are the same settings used by landmark navigation, not separate simple
OpMode limits. The supplied profiles use 3-degree/1-inch drift limits and a
60-second timeout, with 40% final-approach power for both robots.
It stops for invalid tracking, excessive heading or sideways drift, no progress,
or timeout. Driver Station STOP also stops it; gamepad buttons do not cancel this
autonomous. It makes no turns and has
no obstacle avoidance; keep the path clear and watch for wall contact. This
does not determine parking overlap or whether the whole chassis is in the zone;
that depends on your starting placement.

## Landmark travel test

Select **Landmark Travel Test** in **2 - (Tests)**. INIT uses the existing
alliance and audience/scoring start selector to seed the same OTOS + hub-IMU
field estimate. After START, D-pad up/down selects among the twelve
alliance-agnostic flower, garden, parking, and shooting-start destinations plus
the separately saved home. The INIT configuration
table is hidden after START. Navigation status and current/goal poses appear
above the concise landmark list; long status messages wrap onto short lines so
simulator telemetry exposes the complete stop reason. Measured motion reports
robot-center speed and hub-IMU turn rate; OTOS raw velocity reports the sensor's
untransformed X/Y readings from the same update. Destinations describe robot poses separately from
flower attachment points, hive objects, and field regions. Routes use the
profile's chassis geometry, a conservative expanded frame keep-out, and
heading-aligned straight retreat/final-approach segments, with fixed-heading
clearance-checked corridor moves where a garden lies alongside another wall.
Transit corners use quadratic Pedro Bezier curves only when the entire
control-point hull clears the expanded keep-out; otherwise they retain the
checked straight route. Transit paths are chained without stopping at curved
corners; the robot settles before entering constrained wall corridors.
Both profiles temporarily set `bypassSettleTranslationSpeed` to true to diagnose
elevated OTOS velocity readings. Landmark navigation in Landmark Travel Test,
Leave & Park, and Auto Shoot ignores translation speed when settling.
The robot can advance while still translating: keep the route clear
and be ready to press Y. Position, heading, turn-rate, valid-motion, route and
timeout checks remain enabled. Other stationary checks, including Auto Shoot's
shooting checks, are unchanged. Set the flag false to restore the speed check.
At a transit waypoint inside the position tolerance but outside heading
tolerance, `transitAlignmentPower` commands a rotation in place until the
heading is within tolerance. StarterBot uses 15% and CompetitionBot uses 40%; boundary and
timeout checks still apply. Verify physical turn direction and response with
the route clear and Y ready. This does not change final-approach alignment checks.
Unsafe, invalid, or
unsupported routes stop the drive and report why; route selection never changes
an active target. No launcher, intake, or vision fusion is used.

Press A once while idle to start; A during motion cancels. B cancels, Y stops
and cancels with highest priority, and moving either stick beyond its deadband
cancels navigation and resumes manual driving immediately at the profiled 50%
transit limit, using the INIT field-/robot-centric selection. Final approaches
are limited to the profiled `finalApproachPower`: 20% for StarterBot and 40%
for CompetitionBot. This cap also applies within the precision distance of a
waypoint and during settling. X cancels and asks
OTOS to recalibrate only while stationary; it does not reset field coordinates.
Options / Start cancels, stops, and saves the current pose as home only when
that pose is safe; it does not modify the twelve mapped destinations or reset
OTOS. Release buttons before START so a held A cannot launch a route.

CompetitionBot has `geometryVerified` set to true based on operator confirmation;
StarterBot leaves it false. Both profiles leave `pedroTuningVerified` false,
so physical autonomous travel is deliberately blocked while manual drive
remains available. Before enabling the remaining flags, verify the physical chassis
dimensions, field-frame/perimeter clearances, drive and heading signs, Pedro
response, endpoint settling, and all routes on the actual robot at low speed.
Legacy profiles without a `navigation` section still work in existing OpModes;
Landmark Travel Test reports that the section is required rather than inventing
navigation tuning. Deploy the updated paired profiles before using this test.
The profiles' 0.5-inch transit and wall clearances, 10-inch precision-slowdown
radius, 1-inch/3-degree arrival tolerance, and speed/settle limits are
experimental values, not surveyed or physically verified measurements. The
clearances leave little margin for tracking error or unexpected obstacles.
`curveInsetIn` limits corner rounding (currently 6 inches);
each proposed curve must still pass full-hull clearance certification.
Arrival requires at most 1 inch and 3 degrees of error and measured center
speed below 1.5 in/s / turn rate below 5 deg/s continuously for 0.2 seconds,
then all drive outputs are zeroed. Tracking/motion loss or clearance violations
stop immediately; routes also stop after 60 seconds total or 4 seconds without
meaningful progress. `simulatorEnabled` permits simulator experiments only.
TeamCode uses Pedro Pathing 2.1.2; the simulator compiles against its bundled
2.1.12 library. Successful compilation does not establish that physical tuning
is valid.

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
Shooter-ready rumble goes to the gunner; target-lock warnings go to the driver.

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
Drive speed. While aiming with A, the existing 35% translation and full rotation
behavior takes precedence over both settings.
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
