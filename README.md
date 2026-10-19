![Betaflight](https://raw.githubusercontent.com/betaflight/.github/main/profile/images/bf_logo.svg#gh-light-mode-only)
![Betaflight](https://raw.githubusercontent.com/betaflight/.github/main/profile/images/bf_logo_dark.svg#gh-dark-mode-only)

[![Latest version](https://img.shields.io/github/v/release/betaflight/betaflight)](https://github.com/betaflight/betaflight/releases) [![Build](https://img.shields.io/github/actions/workflow/status/betaflight/betaflight/push.yml?branch=master)](https://github.com/betaflight/betaflight/actions/workflows/push.yml) [![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0) [![Join us on Discord!](https://img.shields.io/discord/868013470023548938)](https://discord.gg/n4E6ak4u3c)

Betaflight is flight controller software (firmware) used to fly multi-rotor craft and fixed wing craft. Betaflight focuses on flight performance, leading-edge feature additions, and wide target support.

## Release Schedule

| Date       | Release | Stage             | Status    |
| ---------- | ------- | ----------------- | --------- |
| 26-12-2025 | 2025.12 | Release           | Completed |
| 01-04-2026 | 2026.6  | Beta              |           |
| 01-05-2026 | 2026.6  | Release Candidate |           |
| 01-06-2026 | 2026.6  | Release           |           |
| 01-10-2026 | 2026.12 | Beta              |           |
| 01-11-2026 | 2026.12 | Release Candidate |           |
| 01-12-2026 | 2026.12 | Release           |           |

## News

### 📣 Announcement: New Versioning Scheme & Release Cadence 📣

To create a more predictable release schedule, we're moving to a new versioning system and development cycle, starting with the next release.

**New Format**: `YYYY.M.PATCH` (e.g., `2025.12.1`)

**Release Cadence**: Two major releases per year.

**Target Months**: June and December.

This means the successor to our current `4.x` series will be Betaflight `2025.12.x`, followed by Betaflight `2026.6.x`. We will also align the Betaflight App and Firmware to the same `YYYY.M.PATCH` releases (and cadence).

**Our New Release Cycle**

To support this schedule, our development phases will be structured as follows:

**Alpha**: For new feature development. Alpha builds for the next version will be available shortly after a stable release is published.

**Beta**: A one-month feature freeze for bug fixes only, and existing pull requests currently being reviewed, starting approximately two months before a release.

**Release Candidate (RC)**: A one-month period for final stabilization and testing before the official release.

### Requirements for the submission of new and updated target configuration

The requirements for pull requests adding new targets or modifying existing targets are available on the [betaflight.com website](https://www.betaflight.com/docs/development/manufacturer/requirements-for-submission-of-targets).

## Features

Betaflight has the following features:

- Multi-color RGB LED strip support (each LED can be a different color using variable length WS2811 Addressable RGB strips - use for Orientation Indicators, Low Battery Warning, Flight Mode Status, Initialization Troubleshooting, etc)
- DShot (150, 300 and 600), Multishot, Oneshot (125 and 42) and Proshot1000 motor protocol support
- Blackbox flight recorder logging (to onboard flash or external microSD card where equipped)
- Support for targets that use the STM32 F4, G4, F7 and H7 processors
- PWM, PPM, SPI, and Serial (SBus, SumH, SumD, Spektrum 1024/2048, XBus, etc) RX connection with failsafe detection
- Multiple telemetry protocols (CRSF, FrSky, HoTT smart-port, MSP, etc)
- RSSI via ADC - Uses ADC to read PWM RSSI signals, tested with FrSky D4R-II, X8R, X4R-SB, & XSR
- OSD support & configuration without needing third-party OSD software/firmware/comm devices
- OLED Displays - Display information on: Battery voltage/current/mAh, profile, rate profile, mode, version, sensors, etc
- In-flight manual PID tuning and rate adjustment
- PID and filter tuning using sliders
- Rate profiles and in-flight selection of them
- Configurable serial ports for Serial RX, Telemetry, ESC telemetry, MSP, GPS, OSD, Sonar, etc - Use most devices on any port, softserial included
- VTX support for Unify Pro and IRC Tramp
- and MUCH, MUCH more.

## Installation & Documentation

See: https://betaflight.com/docs/wiki

## Custom-link local odometry

Custom firmware builds with `USE_CUSTOM_LINK` and `USE_GPS` add a 100 Hz
`0x14` odometry downlink alongside `0x11`, without modifying existing payloads.
Its 64-byte payload carries ARM-relative ENU fused position/velocity, FLU
attitude/angular rates, covariance and per-flight origin/validity metadata,
including yaw captured at the successful ARM edge. See
[the protocol definition](src/main/telemetry/custom_link_protocol.h).
The GPS origin is captured at the successful ARM edge with a fresh 3D fix;
GPS loss pauses the stream and recovery keeps that origin. A new ARM resets it.
No 3D fix at ARM means no odometry for that flight, rather than a late invented
origin. Position-source settings and estimator validity/freshness remain in force.
Old hosts can ignore the new ID. A companion bridge can publish these frames as
ROS 2 `nav_msgs/msg/Odometry` on `/fpv/odom` at the same 100 Hz rate, rotating
the pose into a fixed frame with X along the initial ARM heading, Y left and
Z up. Internal flight-navigation/estimator coordinates stay ENU. GPS recovery
keeps both the original position and heading; re-arming captures new ones.

### AirSim POS HOLD odometry regression

The `0x10` and `0x14` attitude/ARM yaw both use **CCW-positive mathematical
yaw from North**. Native `attitude.values.yaw` is a clockwise compass heading
and is negated at the telemetry boundary. Do not send native compass yaw
unchanged in `0x14`: it mirrors the local heading frame at nonzero ARM headings
and makes pose yaw inconsistent with body angular velocity.

Virtual GPS DOP is scaled by 100 (1.0 is 100, not 10). Values below 100 are
treated as unknown by the position estimator and previously inflated GPS
measurement noise by 100x, causing velocity/position lag and slow hold settling.
The virtual provider now supplies valid pDOP/hDOP/vDOP reference values.
The altitude estimator also keeps a filtered disarmed barometer reference at
ARM rather than making one noisy pressure sample a permanent flight-height
offset. This changes reference capture, not the high-rate motion estimate.

The synthetic magnetic field has **2 degrees east declination**. With SITL
stopped, apply the simulator-only calibration to its working EEPROM:

```bash
obj/main/betaflight_SITL.elf --config src/test/sitl/airsim_compass.cfg
```

This sets only `mag_declination=20` (0.1 degree units), not PID gains or the
hardware compass calibration. Keep a backup of the simulation EEPROM. Start
SITL, AirSim and the rebuilt ROS custom-link bridge, stop other controllers,
then in a sourced ROS workspace run:

```bash
python3 src/test/sitl/airsim_odom_hold_test.py \
    --output /tmp/airsim_hold_run --turn --verify
```

The test arms via native RC in ACRO (required by firmware arming checks), selects
POS HOLD+ALT HOLD before taking off, then climbs, flies forward,
brakes, hovers, tests yaw, lands and confirms disarming. All FC control is sent
via the custom protocol; AirSim ground truth is only used for observation.
CSV/JSON results distinguish true lateral motion from odometry frame errors.
Regression limits include X/Y position RMS error below 0.12/0.03 m, settled
horizontal hover span below 0.25/0.10 m, and roll/pitch span below 1 degree.
Use a fresh idle SITL instance for each run; the firmware deliberately latches
RC/API source authority after a pilot disarm, so do not bypass that latch with
repeated old RC commands. The test verifies that its mode commands are accepted
before arming.

Real UE/AirSim regression on 2026-10-12, including hold-mode takeoff, forward
flight, braking, 20 s hover, yaw and landing, passed the above limits at multiple
nonzero ARM headings without changing PID gains. Representative observations:

| Metric | Before corrections | Corrected hold-mode takeoff run |
|---|---:|---:|
| Forward X position RMS error | 0.157 m | 0.089 m |
| Forward Y position RMS error | 0.048 m | 0.008 m |
| Post-flight hover Y position RMS error | 0.095 m | 0.016 m |
| Hover true X/Y excursion | 0.608 / 0.131 m | 0.156 / 0.020 m |
| Hover pitch excursion | 1.661 deg | 0.598 deg |

AirSim's default GPS sensor still has 0.2 s `UpdateLatency`, so moving-position
samples are not exact zero-latency ground truth; the test reports the remaining
error rather than masking it. Altitude mode-entry transients are distinguished
from steady hover by separate CSV stages.

### AirSim balloon interception mission

[airsim_intercept_mission.py](src/test/sitl/airsim_intercept_mission.py) runs
SITL, `apm_bridge/sitl.launch.py`, the real YOLO/NanoTrack `airsim_tracker`,
and `drone_interceptor/launch_relative_imm_ukf.py` on ROS domain 77.
Use the calibrated simulation EEPROM, a rebuilt ROS workspace, and a running
Cosys-AirSim environment with `Copter` and `TargetDrone`. AirSim connects to the
bridge on TCP 5764; the bridge alone connects to SITL on 5763.

Example for the RuralAustralia scene's open road launch point (run from this
repository; other scenes need their own unobstructed launch coordinates):

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/fpv_ws/install/setup.bash
/usr/bin/python3 src/test/sitl/airsim_intercept_mission.py \
    --binary obj/main/betaflight_SITL.elf --eeprom eeprom.bin \
    --ros-ws ~/workspace/fpv_ws \
    --launch-x 100 --launch-y -30 \
    --balloon-x 0 --balloon-y 70 --balloon-z -45 \
    --balloon-motion hover --pitch-aim -57.3 \
    --no-rate-control-attenuation
```

The balloon offsets are relative to the actual grounded interceptor origin.
Its motion script converts the common-world standoff to the target vehicle's
spawn-local coordinates and verifies arrival. The mission aligns ground yaw,
uses gimbal AIM, waits with ARM low until all firmware arming checks remain
clear for 0.5 s, arms through the pilot switch (the bridge auto-publishes
tracker `ON` on the arm edge, verified by the runner), and
requires a sustained visual lock. The cloned run configuration sets
`pwr_on_arm_grace=5`; the runner waits out boot grace rather than bypassing it. A short body-rate/thrust takeoff settles at
4 m before the `MISSION` trigger hands control to relative IMM-UKF guidance.
Ground truth is used for setup, this takeoff handover, and scoring, not as
target-position input to the vision-only guidance; radar is disabled.

The per-run parameter file preserves the packaged profile, then applies the
selected closing-speed and climb/descent ceilings, a 1.5/s climb gain,
55/65 degree mid-course/terminal tilt limits and a 2 rad/s body-rate limit.
The runner defaults to 20/20 m/s ceilings and turn-cap factor 0.8.
The turn-radius governor and image-size proximity cap lower terminal speed
below the cruise ceiling. `control.rate_control_attenuation` is
**false** by default for Betaflight's inner rate loop; use
`--rate-control-attenuation` for an A/B comparison. Keep `use_simulation=false`
so normal collision detection remains active. These are simulation settings,
not certified hardware tuning.

Success requires a **mission-phase common-world 3D distance <= 1 m**, followed
by sixth-detent GPS Rescue, fresh upward-facing contact at the home landing
location (not an earlier target contact aloft), landing within 8 m of home,
speed below 0.3 m/s, and automatic disarming. Sustained tracking loss ends the
engagement and requests the same return path; a miss or failed return exits
nonzero. The runner never kills unrelated processes, modifies the source
EEPROM, or overwrites an existing artifact directory.

The artifact directory contains `report.json`, the full-precision
`trajectory.csv` (both vehicles' world positions, tracker/mission/mode states,
bbox width and LOS age), a camera image, the effective interceptor parameters,
and each process's log. Return-flight proximity cannot relabel a failed
engagement as a hit.

Real UE/Cosys-AirSim run on 2026-10-13, attenuation disabled: balloon horizontal
distance **70.52 m**, height **45.32 m**, mission closest approach **0.882 m**
after **39.02 s**. GPS Rescue returned, landed **0.0097 m** from the launch point,
and auto-disarmed with confirmed ground contact after **123.16 s**.

#### Interactive pilot mode

`--interactive` hands the arm/mission/return decisions to a pilot on a physical
radio: the runner still provisions everything (SITL, bridge, tracker,
interceptor, balloon, alignment), then launches
[sitl_joystick.py](src/test/sitl/sitl_joystick.py) itself - or adopts a bridge
already streaming on UDP 9004 - and supervises instead of flying:

```bash
/usr/bin/python3 src/test/sitl/airsim_intercept_mission.py \
    --binary obj/main/betaflight_SITL.elf --eeprom eeprom.bin \
    --ros-ws ~/workspace/fpv_ws \
    --launch-x 100 --launch-y -30 \
    --balloon-x 0 --balloon-y 70 --balloon-z -45 \
    --interactive --joystick-device /dev/input/js0
```

Pilot controls (the ARM button is momentary: pass
`--joystick-args "--arm-mode latch"` to make each press toggle the switch
level, or hold the button for level semantics):

- **ch7 AUX3** - arm / disarm. The runner verifies the bridge's arm-edge
  tracker `ON` and fails on disarm while airborne.
- **ch8 AUX4** - hold >= 0.5 s: a locked, idle tracker state starts a MISSION
  (the bridge's native RC-trigger path publishes it and requests FC OFFBOARD -
  no aux toggle needed); a second pulse dismisses a running mission - the
  interceptor cancels control while the tracker keeps running, so a later
  pulse can re-trigger once still locked.
- **ch5 AUX1 dial detent 6** - one-key GPS-Rescue RTH at any moment, including
  mid-mission (the firmware strips host authority the instant that switch is
  active); the runner releases OFFBOARD and verifies the return/landing.
- Sticks fly manually whenever no mission is engaged; hover above 2 m before
  the first MISSION (ground-launch guidance can trip the interceptor's
  collision detector - the runner warns below 2 m).

Watchdogs (safety envelope, non-target collision cutoff, disarm-aloft) and the
report/scoring stay those of the automated run; a session passes when every
triggered mission hit (a pilot RTH abort is neutral) and any used RTH returned.

Offline regressions, in the same sourced ROS environment:

```bash
cd src/test/sitl
/usr/bin/python3 -m unittest airsim_intercept_mission_unittest \
    airsim_launcher_unittest airsim_control_test_unittest airsim_rescue_test_unittest \
    sitl_joystick_test
```

Offline regressions cover strict hit/return scoring, measured speed
reporting, home-contact location, duplicate vision timestamps, and launcher
separation/landing alignment. The controller's stopping, scale-invariant closure,
and body-frame gyro integration math also have a standalone host check:

```bash
g++ -std=c++17 -DNDEBUG -I/usr/include/eigen3 -Wall -Wextra -Werror \
    ~/workspace/fpv_ws/src/drone_interceptor/test/guidance_limits_test.cpp \
    -o /tmp/bf_guidance_limits_test
/tmp/bf_guidance_limits_test
```

### Camera8 auto-aiming virtual launcher

The project's editor camera labelled **Camera8** is the actor
`CineCameraActor_0` (saved UE position `(-40608, 178, 185)` cm). The launcher
reads its runtime common-world pose; it does not assume the PIE and standalone
NED origins are the same. Apply the tracker package's `airsim_settings.json`
to the simulator settings and restart the rebuilt UE plugin first. It adds
the `LauncherOverview` camera and three real geometry-query range sensors.
The `LauncherOverview` overhead view is debug-only and **off by default** for
performance (settings check, pose RPCs and a PNG capture per run); pass
`--launcher-overview` to frame and capture it into `launcher_overview.png`.

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/fpv_ws/install/setup.bash
/usr/bin/python3 src/test/sitl/airsim_intercept_mission.py \
    --binary obj/main/betaflight_SITL.elf --eeprom eeprom.bin \
    --ros-ws ~/workspace/fpv_ws \
    --launcher --reference-camera CineCameraActor_0 --launcher-forward 8 \
    --balloon-x 70 --balloon-y 0 --balloon-z -45 \
    --guidance-speed 20 --climb-speed 20 --turn-cap-factor 0.8 \
    --no-rate-control-attenuation
```

Run this command with `--aim-only` first to check placement, pose/IMU/gravity
agreement, obstacle clearance and real visual lock **without arming or releasing**.
The runner resets the simulation before each trial. Balloon offsets rotate with
Camera8's heading and the balloon starts beside the launcher rather than at an
unrelated vehicle origin. Arrival requires both position and sustained low speed.

#### Static and moving balloon targets

The default `--balloon-motion hover` retains a stationary target. To intercept
a moving target, add the following arguments to the Camera8 launcher command:

```bash
    --balloon-motion orbit --balloon-orbit-radius 5 --balloon-orbit-speed 1.5
```

`--balloon-x/y/z` specify the orbit **center** relative to the original launch
position, not the circle's first point. Radius is in metres and orbit speed is
the requested tangential speed in m/s. The default radius/speed are 5/1.5;
the runner accepts positive radii up to 10 m and speeds up to 5 m/s. These
bounds keep the small-area test and its geometry scan finite; they do not
certify interception at every accepted setting.
The target first settles at the standoff, moves to a checked circle entry point,
then follows a simulation-time circular position/velocity reference. Position
feedback corrects drift; logging never pauses control. The entire sampled
circle and its entry connector are checked for geometry clearance before arming
the target. Collision, excessive tracking error or a failed native flight task
ends the run explicitly. This is bounded target motion, not general obstacle
avoidance or a guarantee for arbitrary radii/speeds.

While the Copter is still constrained on the platform, its turntable and pitch
cradle continuously follow the moving target; the camera's AIM offset remains
fixed at -10 degrees. Before MISSION, the runner requires 3 s of visual lock,
camera error below 5 degrees, and independently measured target motion. Static
mode retains the 1 s lock requirement. The platform releases only on the first
interceptor command; it never teleports or steers the airborne interceptor.

Hover and orbit use the same packaged `relative_imm_ukf_params.yaml` controller
profile. The runner applies CLI speed/rate-attenuation limits and the direct-SITL
attitude source; it does not select controller tuning from `balloon_motion`.
Target position/velocity ground truth remains confined to launcher setup and
independent experiment scoring. Online adaptation uses measured LOS and IMU/UKF
estimates, not the scenario's motion label.

Both modes use `target_axial_feedforward_limit=3` m/s. The algorithm estimates
target axial speed from IMU own velocity plus UKF relative velocity, rejects
estimates outside +/-3 m/s, and filters compensation over 0.5 s once bbox width
is at least 80 pixels. Outliers contribute zero rather than a false retreat
velocity. The shared image/lateral floors are 2.6 m/s and 0.4.

The turn-radius and image caps still reduce mid-course speed. After **both** caps,
`terminal_speed_floor=12` m/s prevents proximity-only terminal braking when the
current bbox width is at least 40 pixels and the absolute measured LOS azimuth
rate is at most 0.15 rad/s. The floor never exceeds `v_des`; zero disables it.
Without sufficient visual evidence or with a faster-changing LOS, the ordinary
caps apply. The floor deliberately relaxes the range-based speed cap on a stable
approach; it is a commanded closing-speed floor, not a guarantee of actual speed
or maneuverability. Acceleration jerk, attitude, FOV and body/reference rate
limits remain active. These pixel thresholds were tested at 1536x960 and 72° HFOV.

`vertical_brake_accel=0` disables the optional vertical stopping envelope for
fly-through interception; its 0.5 s delay setting is inactive. Height feedback,
axial/vertical closure coordination, climb/descent limits and vertical acceleration
limits remain active. Independent common-world scoring still requires <=1 m;
no hit or loss threshold was relaxed.

Reports include `prelaunch_tracking`, the configured motion, measured target
speed/displacement/path length, and platform yaw/pitch changes. The CSV records
target velocity and platform angles alongside the existing target positions and
camera error. A successful moving-target test must show actual motion, not just
`motion=orbit` in its configuration.

To isolate a test from unrelated ROS nodes, use a free domain for the **whole**
runner, for example:

```bash
ROS_DOMAIN_ID=78 /usr/bin/python3 src/test/sitl/airsim_intercept_mission.py \
    --binary obj/main/betaflight_SITL.elf --eeprom eeprom.bin \
    --ros-ws ~/workspace/fpv_ws --launcher \
    --balloon-x 70 --balloon-y 0 --balloon-z -45 \
    --balloon-motion orbit --balloon-orbit-radius 5 --balloon-orbit-speed 1.5 \
    --guidance-speed 20 --climb-speed 20 --turn-cap-factor 0.8 \
    --no-rate-control-attenuation
```

The runner passes that domain to its bridge/tracker/controller children; without
an environment override it uses 78. This does not permit simultaneous missions
to control the same AirSim vehicles or SITL ports.

The historical 2026-10-15 dual-mode acceptance used PN gain 3.5,
terminal tilt 65 degrees and body/reference rate limits of 2 rad/s as per-run
overrides. These values are now in the shared packaged profile; the historical
results below precede the observed terminal speed floor.
With guidance/climb ceilings 20/20, horizontal standoff 70 m and height 45 m,
the final same-UE static and orbit runs both intercepted within 1 m and returned
to fresh home contact with automatic disarming. Earlier dynamic tuning misses
remain failures; this is not a success guarantee for arbitrary target speeds.

| Acceptance case | Closest distance (m) | Intercept time (s) | Target path during engagement (m) | Home XY error (m) |
| --- | --- | --- | --- | --- |
| Hover | 0.814489 | 6.779 | 0.00 | 0.01184 |
| Orbit, requested 5 m / 1.5 m/s | 0.966386 | 14.866 | 24.65 | 0.02932 |

The moving target's measured speed at closest approach was 1.66 m/s; physical
tracking is approximate, so the configured orbit speed/radius are references,
not exact measured values. During prelaunch visual lock the target moved 6.0 m,
the platform changed yaw/pitch by 4.79/1.02 degrees, and camera error stayed
below 0.11 degrees. The final orbit hit occurred during a short terminal tracking
dropout under the existing coast/2 s loss policy; neither that policy nor the
1 m hit criterion was relaxed. Two earlier moving-target confirmations also hit
at 0.944/0.964 m and returned.

A subsequent repeat intercepted at 0.934 m but GPS Rescue aborted during the
return. That report remains a **failed full-loop run**, not a successful return;
this target-motion feature does not certify return-controller reliability.

#### Shared-profile terminal speed acceptance

On 2026-10-16, three consecutive final-profile missions used the same UE process
without restarting it. All intercepted within 1 m, returned to fresh home ground
contact, automatically disarmed and released target API control. Hover and orbit
at 20/20 had identical generated algorithm parameters. The 40/20 run changed only
the requested cruise ceiling; no target-mode-specific tuning was applied.

| Target | Guidance/climb ceilings (m/s) | Closest distance (m) | True speed near 10 m / 3 m (m/s) | Intercept time (s) | Home XY error (m) |
| --- | --- | --- | --- | --- | --- |
| Hover | 20/20 | 0.615 | 14.17 / 13.91 | 6.1 | 0.0334 |
| Orbit, 5 m / 1.5 m/s reference | 20/20 | 0.853 | 13.14 / 12.79 | 6.9 | 0.0367 |
| Orbit, 5 m / 1.5 m/s reference | 40/20 | 0.635 | 13.34 / 13.29 | 6.7 | 0.0133 |

Speeds are AirSim ground-truth velocity magnitudes from the mission CSV samples
nearest 10 m and 3 m target distance, before RTH. They are not the configured
speed ceilings or IMU speed estimates. In the preliminary 3 m/s-floor orbit run,
these speeds were 7.56 / 3.60 m/s; that tuning was rejected for terminal slowdown.
The final runs show substantially preserved terminal speed in this geometry,
not validation for every moving target or proof that 40 m/s flight was attained.
They retain the direct-attitude simulation limitation described below; no
firmware source, firmware binary or source EEPROM was changed for this tuning.

#### Dynamic repeatability with direct SITL attitude

The 2026-10-16 dynamic validation uses the existing `SITL_ATTITUDE_DIRECT`
build option. In the failing normal build, the FC attitude diverged from
AirSim attitude during a terminal miss and GPS Rescue subsequently aborted.
The underlying Mahony estimator divergence remains unresolved. Direct attitude
is a simulation workaround for testing guidance and return with a consistent
ownship attitude, not validation of the normal onboard attitude estimator.

Build explicitly before reproducing these results (clean when changing build
flags, because an existing object may otherwise retain its previous mode):

```bash
make TARGET=SITL clean
make TARGET=SITL EXTRA_FLAGS=-DSITL_ATTITUDE_DIRECT -j4
```

The mission overlay sets `estimation.gyro_attitude_during_mission=false` to
consume the FC attitude; the generic packaged profile keeps gyro propagation
enabled. The runner accepts any supplied binary and does not detect its build
mode. A later clean build without this flag restores the normal estimator and
is outside the following acceptance evidence.

Three consecutive trials in the same UE process used the Camera8 launcher,
70 m / 45 m standoff, requested orbit radius 5 m and speed 1.5 m/s, guidance/climb
ceilings 40/20 m/s, turn-cap factor 0.8 and rate attenuation disabled:

```bash
/usr/bin/python3 src/test/sitl/airsim_intercept_mission.py \
    --binary obj/main/betaflight_SITL.elf --eeprom eeprom.bin \
    --ros-ws ~/workspace/fpv_ws --launcher \
    --reference-camera CineCameraActor_0 --launcher-forward 8 \
    --balloon-x 70 --balloon-y 0 --balloon-z -45 \
    --guidance-speed 40 --climb-speed 20 --turn-cap-factor 0.8 \
    --no-rate-control-attenuation \
    --balloon-motion orbit --balloon-orbit-radius 5 --balloon-orbit-speed 1.5
```

| Trial | Mission closest distance (m) | Intercept time (s) | Home XY error (m) | Return |
| --- | --- | --- | --- | --- |
| 1 | 0.953428 | 8.995 | 0.03323 | Fresh ground contact, auto-disarmed |
| 2 | 0.935805 | 8.989 | 0.02989 | Fresh ground contact, auto-disarmed |
| 3 | 0.899324 | 10.928 | 0.03569 | Fresh ground contact, auto-disarmed |

Measured peak horizontal/climb speeds were 13.88/12.26 m/s across these runs;
40/20 are configured ceilings, not attained flight speeds. The target reached
1.66 m/s and moved 14.86–18.08 m during engagement. UE was left running, the
target was parked with API control released, and the source EEPROM hash was
unchanged. The 1 m success threshold and return requirements were unchanged.

Focused validation passed: 48 mission runner tests, 14 launcher tests and the
standalone guidance/IMU helper regression. Broader Python discovery had one
pre-existing Rescue profile expectation failure (expected ground speed 250,
configured 300); the full Python suite is not reported as passing.

#### Repeating trials in one UE process

Keep UE in game/Play mode; rerun the same mission command with a new artifact
directory (or let the PID-based default choose one). The runner does not close,
restart, or rebuild UE. `--rpc-wait 30` bounds startup readiness checks and names
the host/port in a failure; a closed game or RPC server must still be started.
Stale ROS discovery entries are not sufficient readiness evidence: the runner
also requires the new interceptor's successful initialization log.

Each trial cancels the target's previous native task before reset. Balloon setup
parks/disarms the target, zeros its complete kinematics, applies the requested
scale even when it is 1, and checks a sampled 27-ray corridor around candidate
staging points. It climbs vertically before translating at altitude. If the
preferred launch column is blocked, nearby columns/heights are checked; if none
is clear, the target is not armed. Collision/stagnation and failed RPC commands
are explicit errors, not six repeated blind pushes against an obstacle.

Short velocity tasks use RPC `Future.get()`, not `join()` (the latter waits but
does not raise a failed/timed-out result in this SDK). Arrival still requires
less than 1 m error and sustained speed below 0.25 m/s. Every exit cancels the
UE-side task, parks the disarmed balloon with `airsim.LauncherLock.TargetDrone 1`,
and releases API control; cleanup never flies toward a hardcoded old spawn.
The next reset clears the parking constraint. These are simulation preparation
and cleanup operations, not target-position inputs to interception guidance.

The launcher's spawn-frame offset is remeasured **after** the reset fall has
settled. Measuring the physics and rendered pose during the fall produced a
spurious approximately 0.2 m offset in some repeated runs.

Firmware/EEPROM changes can also invalidate previously tested tuning. The
mission now provisions profile 0 explicitly in its **cloned** EEPROM:

| Axis | P | I | D | F |
| --- | --- | --- | --- | --- |
| Roll | 37 | 62 | 22 | 95 |
| Pitch | 32 | 57 | 22 | 84 |
| Yaw | 10 | 0 | 0 | 8 |

`simplified_pids_mode=OFF` prevents re-synthesis of these calibrated values.
The runner reads P/I/D back with MSP before starting the flight stack and aborts
on a mismatch. Reports record the measured gains plus firmware/source-EEPROM
hashes. This is **SITL-only tuning**, never a hardware recommendation, and the
source EEPROM is not overwritten.

The rendered-camera overlay uses capture-time bbox/LOS
(`publish_prediction:=False`), timestamp-matched camera orientation in IMM-UKF,
and a bounded 0.25 s relative-state prediction to the control epoch. The general
tracker still defaults to prediction mode. The simulation gimbal uses tracking
gain 2, maximum rate 60 deg/s, acceleration 180 deg/s², filter 0.12 s, and
`tracking_rate_feedforward=0`; body-motion stabilization remains active.
Other gimbal profiles retain the default rate-feedforward gain of 1. A sustained
large mid-course boresight error terminates the test and requests Rescue rather
than allowing continued blind flight; this scoring/safety check does not steer
the controller using ground truth. Near-pass camera error can grow as the target
fills/leaves the frame and is reported separately from the mid-course maximum.

After changing the ROS sources, rebuild the affected packages in the sourced
workspace; no UE restart is needed:

```bash
cd ~/workspace/fpv_ws
colcon build --packages-select apm_bridge --cmake-target custom_link_bridge_node
colcon build --packages-select drone_interceptor --cmake-target interceptor_node_relative_imm_ukf
colcon build --packages-select airsim_tracker --symlink-install
```

On 2026-10-15 the final repeated-run profile completed three consecutive
20/20 m/s ceiling trials in the **same UE process**, without restarting the game:

| Run | Closest 3D distance (m) | Intercept time (s) | Landing XY error (m) | Return |
| --- | --- | --- | --- | --- |
| 1 | 0.846894 | 6.443 | 0.01346 | Grounded, auto-disarmed |
| 2 | 0.955311 | 6.411 | 0.01013 | Grounded, auto-disarmed |
| 3 | 0.828669 | 6.611 | 0.00422 | Grounded, auto-disarmed |

All three retained target tracking through the intercept, passed the unchanged
1 m criterion, and left the target parked with API control disabled. The third
run's recorded maximum mid-course camera error (range greater than 12 m) was
5.74 degrees. Earlier debug misses are preserved as failures, not included in
this final matrix. A separate injected obstacle at the preferred balloon staging
point was rejected and an alternate column succeeded; a blocked endpoint failed
before arming and cleanup still completed. An absent RPC endpoint produced the
bounded endpoint-specific diagnostic rather than an immediate opaque retry error.

#### Fast Rescue descent and touchdown detection

`gps_rescue_descend_rate=200` and `gps_rescue_landing_rate=50` request 2 m/s
descent above the Rescue return-altitude boundary and 0.5 m/s below it.
With `gps_rescue_return_alt=8`, braking at that boundary can briefly stop the
vehicle around 6.7 m. That is not touchdown. Once descent had been established,
the old detector allowed quiet-time accumulation at any altitude and could
auto-disarm during this transient.

The shared mission/geofence/Rescue landing detector now requires the vehicle to
remain below `ap_landing_altitude` (default 4 m) before accumulating touchdown
quiet time. Leaving that near-ground envelope resets the timer. The descent-
established check and sustained stopped-descent/vertical-velocity conditions
still apply. The 8 m speed-transition boundary is **not** a landing-completion
height; do not raise the landing envelope to match it. The simulation runner
continues to require fresh physical home contact and a settled, grounded disarm.

Two same-UE full-loop trials on 2026-10-15 retained the 200/200/50 cm/s
ascend/descent/landing profile and 300 cm/s return speed. Both stayed armed
through the stationary 6.7-6.8 m braking transient and landed with confirmed
contact before auto-disarming. Mean descent was 1.95-1.97 m/s above 10 m and
0.55-0.57 m/s during the final 1.5-4 m interval. Return durations were
71.09/71.93 s; landing XY errors were 0.0099/0.0033 m. The fix is in the
firmware: rebuild with `make TARGET=SITL` after changing it; no UE restart is
needed.

The generated base, pedestal, turntable, saddle and pitch cradle use existing UE
cube meshes with measured unit-size normalization. AIM is fixed at
`pitch-aim=-10`: the camera requests 80 degrees relative to the body, and the
launcher turns/tilts the body to align that boresight with the target. The
virtual joint's 85-degree limit and simulated 65-degree arming allowance are
**SITL only**, not changes to the hardware model's default mechanical limits.

`airsim.LauncherLock.Copter 1` constrains only the mounted Copter while its real
sensor/controller updates continue with supporting gravity reaction. The
first interceptor body-rate command releases it with `airsim.LauncherLock.Copter 0`;
normal flight physics and collision response then apply. The cradle levels only after at least 5 m separation; its landing saddle is then
centered under the original mounted launch/home position, not the displaced pivot.
This platform path starts `MISSION` on the mount
instead of using the legacy 4 m hover handover.

The launcher-specific bridge profile allows MISSION gimbal stabilization before
airborne confirmation. The AirSim gimbal retains a world-referenced boresight
through body motion. The tracker discards repeated/out-of-order prediction
timestamps instead of resetting a valid LOS, and the image-capture cache isolates
different cameras and image request types. The interception profile uses
short-horizon IMU axial-speed aiding and disables noisy augmented-PN acceleration
feed-forward; no target ground truth is fed into guidance.

The high-speed profile adds coordinated stopping rather than simply increasing
the two speed ceilings:

- `controller.vertical_brake_accel=6` and `vertical_brake_delay=0.5` bound the
  vertical reference by `v*delay + v*v/(2*accel) <= abs(dz)`.
- `controller.visual_speed_gain=600` caps axial speed by `600/bbox_size_px`,
  independent of monocular range-scale errors. With this option enabled, the
  vertical reference is also bounded by `axial_reference*abs(dz)/horizontal_range`.
  Thus image-based horizontal braking cannot leave the climb running at full
  speed and carry the vehicle above the target. The pixel gain is specific to
  the 1536x960, 72-degree camera; recalibrate it for different optics.
- Axial/vertical velocity gains are 1.5/4, lateral PN gain is 2.5, terminal
  lateral decay is 0.1, and observability dither is reduced to 0.1 m/s^2.
  Axial deadband is 0.05 m/s so low-speed terminal closure does not stall.
  The profile retains perception position/size standard-deviation entries of
  12/3 pixels. The current node does not consume these two YAML fields; its
  internal measurement-noise model remains unchanged.
- Estimated capture range is 0.5 m, not the old 2 m early-hold trigger.
  This does **not** change the independent common-world 1 m hit criterion.

The packaged `relative_imm_ukf_params.yaml` now includes these validated 20/20
Camera8 controller settings, IMU aiding and vision-only mode, plus the retained
perception entries. The `topics` block controls all five input subscriptions and
both output publishers, including the camera-orientation input `topics.gimbal`.
Names are applied at startup and follow normal ROS namespace/remapping rules.
The packaged Bool mission input is `/fpv/mission_start`; without a parameter
file its compatibility default remains `/fpv/mission_launch`. The Header
MISSION/OFF trigger uses `topics.mission_launch` (`/fpv/tracker_trigger`).
Topic-binding regressions can be run in a sourced ROS workspace without AirSim
or a mission trigger:

```bash
ROS_DOMAIN_ID=178 /usr/bin/python3 \
    ~/workspace/fpv_ws/src/drone_interceptor/test/relative_imm_ukf_topics_test.py -v
```

The node's code-level defaults for optional limits remain disabled when no
parameter file is loaded. The runner still writes `interceptor_parameters.yaml`
to record the effective configuration and apply requested speed overrides.
The algorithm YAML alone does not configure the tracker, bridge/gimbal, flight
controller PID tuning, launcher preparation, or test safety supervision.

The packaged profile enables `estimation.gyro_attitude_during_mission`: body-to-world
attitude is seeded from the settled prelaunch FC orientation, then propagated
with the measured body gyro during the short engagement. This avoids feeding
acceleration-induced FC tilt corrections into the companion's gravity subtraction
and velocity integration. Outside MISSION, the companion again uses FC orientation.
Betaflight keeps its normal `imu_dcm_kp=2500` for GPS Rescue and landing; the
temporary 500-gain experiment was not retained because return-flight drift occurred.
Neither simulator attitude nor target ground truth is injected into guidance.
`trajectory.csv` records the FC's ENU quaternion as well as simulator attitude,
and controller logs include IMU axial/vertical speed for comparison.

A three-ray, +/-1 m launch corridor check rejects near obstacles. Any new
non-target collision during the engagement causes an emergency stop, not a
continued blind return attempt. Tracking loss without collision requests Rescue;
the test supervisor also ends a missed pass if common-world range reopens by
more than 8 m from its closest approach. This ground-truth check only aborts the
test and requests Rescue; it is not a guidance input. An aborted Rescue is
reported immediately. These checks are not general-purpose
obstacle avoidance.

The earlier 12/10 m/s Camera8 trial on 2026-10-14 achieved a **0.870 m** closest approach in
**7.75 s**, with **13.32 m/s** peak speed, and landed **0.026 m** from the
launch point. A subsequent settled-hover-target trial also passed the strict
1 m intercept criterion and returned.

The 2026-10-14 high-speed profile was then validated in four consecutive full-loop
trials with a settled hovering balloon approximately 70 m horizontally away and
45 m above launch:

| Guidance/climb ceilings (m/s) | Run | Closest (m) | Intercept (s) | Peak 3D speed (m/s) | Peak climb (m/s) | Landing XY error (m) |
| --- | --- | --- | --- | --- | --- | --- |
| 20 / 20 | 1 | 0.841938 | 6.799 | 18.039 | 10.529 | 0.0201 |
| 25 / 25 | 1 | 0.974968 | 7.245 | 18.856 | 10.924 | 0.0491 |
| 20 / 20 | 2 | 0.858568 | 6.837 | 17.724 | 10.626 | 0.0409 |
| 25 / 25 | 2 | 0.922721 | 6.522 | 18.930 | 11.626 | 0.0252 |

All four final trials met the unchanged 1 m common-world criterion, returned,
made fresh upward-facing contact on the home saddle, and auto-disarmed. No
non-target engagement collision was recorded in this final matrix. Earlier
failed debug trials remain failures and are not included as passes.
Repeat the command above with both speed arguments set to 25 for the second
validated ceiling pair; the rest of the profile is identical.

These numbers distinguish **configured speed ceilings** from **measured speed**.
The short 45 m climb includes acceleration and terminal braking; these tests
do not demonstrate an actual 20 m/s climb or certify other geometries, moving
targets, or hardware. Matching algorithm parameters does not replace the complete
mission workflow and its other component settings.

Artifacts additionally include `aim_camera.png`, `launcher_overview.png`,
`bridge_parameters.yaml`, mounted pose/IMU agreement, clearance distances,
release timing and peak speed.

## Support and Developers Channel

There's a dedicated [Discord server](https://discord.gg/n4E6ak4u3c) for help, support and general community.

## Betaflight Application

To configure Betaflight you should use the [Betaflight App](https://app.betaflight.com). It is a progressive web app, so should always be the latest version.

## Contributing

Contributions are welcome and encouraged. You can contribute in many ways:

- implement a new feature in the firmware or in the app (see [below](#developers));
- documentation updates and corrections;
- How-To guides - received help? Help others!
- bug reporting & fixes;
- new feature ideas & suggestions;
- provide a new translation for the app, or help us maintain the existing ones (see [below](#translators)).

The best place to start is the Betaflight Discord (registration [here](https://discord.gg/n4E6ak4u3c)). Next place is the github issue tracker:

https://github.com/betaflight/betaflight/issues
https://github.com/betaflight/betaflight-configurator/issues

Before creating new issues please check to see if there is an existing one, search first otherwise you waste people's time when they could be coding instead!

If you want to contribute to our efforts financially, please consider making a donation to us through [PayPal](https://paypal.me/betaflight).

If you want to contribute financially on an ongoing basis, you should consider becoming a patron for us on [Patreon](https://www.patreon.com/betaflight).

## Developers

Contribution of bugfixes and new features is encouraged. Please be aware that we have a thorough review process for pull requests, and be prepared to explain what you want to achieve with your pull request.
Before starting to write code, please read our [development guidelines](https://www.betaflight.com/docs/development) and [coding style definition](https://www.betaflight.com/docs/development/CodingStyle).

GitHub actions are used to run automatic builds.

### Building with Docker/Devcontainers

A preconfigured [devcontainer](.devcontainer/README.md) is included for a consistent build environment across all platforms. This is the recommended approach for Windows developers:

```bash
# With VS Code: Install "Dev Containers" extension, open folder, and select "Reopen in Container"

# Or command-line only:
docker build -t betaflight-dev -f .devcontainer/containerfile .devcontainer/
docker run --rm -v "${PWD}:/workspace" -w /workspace betaflight-dev make TARGET=SPEEDYBEEF405WING
```

See the [devcontainer documentation](.devcontainer/README.md) for detailed setup instructions including hardware flashing.

## Translators

We want to make Betaflight accessible for pilots who are not fluent in English, and for this reason we are currently maintaining translations into 21 languages for Betaflight Configurator: Català, Dansk, Deutsch, Español, Euskera, Français, Galego, Hrvatski, Bahasa Indonesia, Italiano, 日本語, 한국어, Latviešu, Português, Português Brasileiro, polski, Русский язык, Svenska, 简体中文, 繁體中文.
We have got a team of volunteer translators who do this work, but additional translators are always welcome to share the workload, and we are keen to add additional languages. If you would like to help us with translations, you have got the following options:

- if you help by suggesting some updates or improvements to translations in a language you are familiar with, head to [crowdin](https://crowdin.com/project/betaflight-configurator) and add your suggested translations there;
- if you would like to start working on the translation for a new language, or take on responsibility for proof-reading the translation for a language you are very familiar with, please head to the Betaflight Discord chat (registration [here](https://discord.gg/n4E6ak4u3c)), and join the ['translation'](https://discord.com/channels/868013470023548938/1057773726915100702) channel - the people in there can help you to get a new language added, or set you up as a proof reader.

## Hardware Issues

Betaflight does not manufacture or distribute their own hardware. While we are collaborating with and supported by a number of manufacturers, we do not do any kind of hardware support.

If you encounter any hardware issues with your flight controller or another component, please contact the manufacturer or supplier of your hardware, or check [Discord](https://discord.gg/n4E6ak4u3c) to see if others with the same problem have found a solution.

## Betaflight Releases

You can find our release [here](https://github.com/betaflight/betaflight/releases) on Github and we also have more detailed [release notes](https://www.betaflight.com/docs/category/release-notes) at [betaflight.com](https://www.betaflight.com).

## Open Source / Contributors

Betaflight is software that is **open source** and is available free of charge without warranty to all users.

For a complete list of contributors (past and present) see [Github](https://github.com/betaflight/betaflight/graphs/contributors).
