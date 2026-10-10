## SITL in gazebo 8 with ArduCopterPlugin
SITL (software in the loop) simulator allows you to run betaflight/cleanflight without any hardware.
Currently only tested on Ubuntu 16.04, x86_64, gcc (Ubuntu 5.4.0-6ubuntu1~16.04.4) 5.4.0 20160609.

### install gazebo 8
see here: [Installation](http://gazebosim.org/tutorials?cat=install)

### copy & modify world
for Ubunutu 16.04:
`cp /usr/share/gazebo-8/worlds/iris_arducopter_demo.world .`

change `real_time_update_rate` in `iris_arducopter_demo.world`:
`<real_time_update_rate>0</real_time_update_rate>`
to
`<real_time_update_rate>100</real_time_update_rate>`
***this suggest set to non-zero***

`100` mean what speed your computer should run in (Hz).
Faster computer can set to a higher rate.
see [here](http://gazebosim.org/tutorials?tut=modifying_world&cat=build_world#PhysicsProperties) for detail.
`max_step_size` should NOT higher than `0.0025` as I tested.
smaller mean more accurate, but need higher speed CPU to run as realtime.

### build betaflight
run `make TARGET=SITL`

### settings
to avoid simulation speed slow down, suggest to set some settings belows:

In `configuration` page:

1. `ESC/Motor`: `PWM`, disable `Motor PWM speed Sparted from PID speed`
2. `PID loop frequency` as high as it can.

### start and run
1. start betaflight: `./obj/main/betaflight_SITL.elf`
2. start gazebo: `gazebo --verbose ./iris_arducopter_demo.world`
4. connect your transmitter and fly/test, I used a app to send `MSP_SET_RAW_RC`, code available [here](https://github.com/cs8425/msp-controller).

### RC from a host joystick (/dev/input/js0)

`src/test/sitl/sitl_joystick.py` bridges a Linux joystick - e.g. an OpenTX/
EdgeTX radio in USB joystick mode - onto the RC input as a replacement for a
receiver link:

    python3 src/test/sitl/sitl_joystick.py --probe     # inspect the device
    python3 src/test/sitl/sitl_joystick.py --print-config > joystick.cfg
    obj/main/betaflight_SITL.elf --config joystick.cfg # save in this working directory
    python3 src/test/sitl/sitl_joystick.py --monitor   # bridge + live status

It sends 50 Hz `rc_packet` frames (double timestamp + 16 x uint16 us) and
maps sticks to AETR, a 6-pos dial to AUX1 (the firmware's native 200 us
bands), a 3-pos toggle to AUX2, an ARM switch exposed as button 0 to AUX3 and a Trigger
button to AUX4; the module docstring carries the matching `aux` provisioning
lines. For interactive flight in the repo's physics harness:
`python3 src/test/sitl/sitl_harness.py --binary obj/main/betaflight_SITL.elf
--scenario manual --joystick /dev/input/js0`. While this bridge streams, a
custom-link HOST_RC (0x23) injector defers to it (see `rxUdpBridgeRcFresh`),
so it also works alongside an AirSim/custom-link host without the two RC
sources fighting.

`sitl_joystick.py` is the shared source of the channel indices and CLI table:
ARM is AUX3, not AUX1. The dial's positions are ACRO / ANGLE / HORIZON /
ALTHOLD / POSHOLD+ALTHOLD / GPSRESCUE at 1000/1200/1400/1600/1800/2000 us.
The fifth detent holds both horizontal position and altitude. ARM follows
the switch level (button value 1 -> 2000 us, value 0 -> 1000 us), including
initial device-state events; it no longer toggles on each press. AUX2 high
requests OFFBOARD; AUX4 remains an unbound Trigger.
For AirSim, also enable GPS with the VIRTUAL provider, set `trust_mag = ON`
and use the appropriate `ap_hover_throttle` (1211 for the calibrated 2 kg
F70/HQ7040 frame, with `ap_throttle_min=1050`).
Apply configuration with SITL stopped, then restart it from the same working
directory so it loads the updated `eeprom.bin`.

### note
betaflight	->	gazebo	`udp://127.0.0.1:9002`
gazebo	->	betaflight	`udp://127.0.0.1:9003`
rc		->	betaflight	`udp://127.0.0.1:9004`

UARTx will bind on `tcp://127.0.0.1:576x` when port been open.

`eeprom.bin`, size 8192 Byte, is for config saving.
size can be changed in `src/platform/SITL/link/SITL.ld` >> `__FLASH_CONFIG_Size`

### Automated control tests with UE / Cosys-AirSim physics

Use `src/test/sitl/airsim_control_test.py` for a real AirSim `BetaFlight`
vehicle. Unlike `sitl_harness.py`, it does not create a motion model or send
FDM/IMU/GPS samples. AirSim provides sensors and physics through the custom
link on TCP 5763; the script sends only 50 Hz AETR RC on UDP 9004 and reads
AirSim ground truth plus MSP on TCP 5761.

Start UE simulation with RPC enabled and the Cosys-AirSim Python client
installed, disable AirSim API control, and stop any existing SITL or RC
feeder. Do not run the UDP FDM harness alongside an AirSim `BetaFlight`
vehicle: both would overwrite the same virtual sensors.

    make TARGET=SITL
    python3 src/test/sitl/airsim_control_test.py \
        --binary obj/main/betaflight_SITL.elf --vehicle Copter --scenario all

The script clones `--eeprom eeprom.bin` (including the calibrated PID profile),
provisions and launches its own SITL in a new artifact directory;
it never kills existing instances or overwrites your normal `eeprom.bin`.
It checks sensor/heading agreement before arming, then takeoff, a sustained
hover, roll/pitch/yaw direction, a 20 m northbound waypoint and landing.
`--scenario sensors` is a non-flying preflight check; `hover` and `mission`
select smaller flight tests. Failures return a nonzero exit status, disarm
the simulated vehicle and stop only the child SITL.

The first eight channels and AUX rules come directly from `sitl_joystick.py`.
The automatic test uses the fifth detent for combined altitude/position hold
and adds only AUTOPILOT on AUX6 for missions, without changing the radio's
mode dial, ARM switch or Trigger. AUX5 remains unused. The additional mission
switch is test-only and is not part of the exported joystick configuration.
Its AUX-rule slot is 7; canonical slot 6 belongs to the sixth-detent Rescue
switch and must not be overwritten by a test mission rule.

Artifacts include `config.txt`, `provision.log`, `sitl.log`,
`trajectory.csv` (AirSim and FC attitude, position, velocity, modes and
arming flags), and `report.json`. Use `--output <new-directory>` to select
their location.

`--hover-pwm` defaults to 1211 for AirSim's 2 kg F70/HQ7040 QuadX with four
22.761 N rotors. At sea level each rotor needs about 21.54% of maximum thrust.
With PWM endpoints 1055..2000 (`motor_idle=550`), mixer/OFFBOARD collective
is 16.976%; `altitudeControl` maps that through 1050..2000 to 1211 us.
Use `ap_throttle_min=1050` for descent headroom and
`airmode_start_throttle_percent=15` so a low-throttle takeoff can latch
wasThrottleRaised and activate the level/hold loops (the default 25% exceeds
the new hover collective). The default linear RC curve
maps 1250 us input to the 1211 us ALTHOLD neutral point; the test uses this
conversion instead of applying the hover parameter directly to the stick.
Different mass/rotors/RC curves/endpoints require matching values. The test enforces
a 15 m height / 45 m horizontal / 45 degree tilt envelope. These are
simulation-only tests, not hardware flight procedures.

The F70 model also has substantially higher roll/pitch thrust and Yaw torque
than the former rotor model. The validated 2 kg frame uses profile-0
roll P/I/D/F=10/17/6/26, pitch=12/21/8/31, Yaw=30/0/0/24, d_max_roll=9,
d_max_pitch=11 and feedforward_yaw_hold_gain=30. Do not run the low-thrust
model's former high gains with the new rotors: measured rates/output can
saturate when POSHOLD starts steering. These are simulation regression
settings, not general hardware defaults; no firmware PID defaults changed.
Navigation regression now requires continuous 60 s waypoint dwell, including
altitude retention after the completion callback clears its target. Landing
uses a bounded velocity-driven altitude reference; its far-below sentinel
must not become a large altitude-P error that forces idle/freefall.

In active ALTHOLD (including the fifth POSHOLD+ALTHOLD detent), full-low
throttle requests descent at `alt_hold_climb_rate`; it is not a command to
hold altitude or disarm. ARM low remains the disarm command. The automatic
test lands with 1000 us throttle while checking that ALTHOLD remains active.
The neutral deadband is still centered on `ap_hover_throttle`; setting
`alt_hold_deadband = 0` disables pilot altitude adjustments. Mission altitude
targets and failsafe descent retain priority over the pilot's throttle.

Offline runner checks:

    python3 -m unittest discover -s src/test/sitl -p airsim_control_test_unittest.py

### Native AirSim OFFBOARD test

Use `src/test/sitl/airsim_offboard_test.py` to verify the actual AirSim API
and custom-link control path, rather than the UDP RC path:

    python3 src/test/sitl/airsim_offboard_test.py \
        --binary obj/main/betaflight_SITL.elf --eeprom eeprom.bin --vehicle Copter

Start UE Play with the vehicle stationary on a valid ground surface, stop
other SITL/RC senders and disable existing API control first. The test clones
the current EEPROM (including PID tuning) into a temporary directory. It
sends no UDP RC/FDM packets and does not modify the source EEPROM.

Set the AirSim vehicle's `RC.RemoteControlID` to `-1` and
`RC.AllowAPIWhenDisconnected` to `true`, then restart UE. This disables
AirSim's native joystick mapping, which would otherwise overwrite RC values
every physics tick even when the external joystick bridge is stopped.
The external `sitl_joystick.py` bridge is unaffected.

It verifies host arming with AUX3 ARM low, OFFBOARD with the firmware's
angle/altitude/position loops inactive, a 2 m takeoff/hover/landing and body-rate tracking in both
Yaw directions plus roll/pitch. Python rate commands are FLU rad/s; AirSim
ground-truth FRD rates negate pitch and Yaw. The script supplies an explicit
small host attitude/altitude loop with horizontal velocity damping using `moveByAngleRatesThrottleAsync`;
the adapter does not implement high-level takeoff/hover/position commands,
and OFFBOARD itself disables the firmware's level/altitude/position loops.

After landing it checks API disarm, re-arm and loss of host arming/OFFBOARD
when API control is disabled. Cleanup disarms via the API before disabling
it and stopping the child SITL. State CSVs, provision/SITL logs and a JSON
report are preserved under `--output <new-directory>`.
Touchdown requires a fresh ground collision and low ground-truth velocity,
not just reaching the launch point's height. Completing a Python rate command
does not stop the adapter's 100 Hz heartbeat: the last command persists while
API control is enabled. Send a neutral command, disarm and disable API control
explicitly when handing control back.

API OFFBOARD uses the `0x20` mode request, not an injected AUX2 switch.
Changing the radio dial preempts API-entered OFFBOARD without requiring
neutral sticks or low throttle. Old API heartbeats cannot take control back;
disable/re-enable API control for an explicit new request. The real AUX2
OFFBOARD switch retains its previous priority over ordinary dial modes.
GPS Rescue preempts both paths and releases host PID/throttle authority
before the Rescue/hold controllers activate. API throttle also counts
towards the existing throttle-raised latch after arming.

An API-armed flight stays armed on pilot takeover even if AUX3 was low.
A subsequent ARM high-to-low edge explicitly disarms, and Rescue's automatic
disarm is not undone by old API arm requests. Pilot RX loss still invokes
normal failsafe; stale HOST_RC fallback does not overwrite the selected mode.

To test all radio detents, explicit switch behavior and API-to-GPS Rescue:

    python3 src/test/sitl/airsim_offboard_test.py \
        --binary obj/main/betaflight_SITL.elf --eeprom eeprom.bin \
        --vehicle Copter --takeover

This adds real UDP RC (never synthetic FDM), requires about 40 m of clear
flight area and keeps the old API heartbeat active through Rescue landing.

### Automatic real-AirSim Yaw tuning

`src/test/sitl/airsim_yaw_tune.py` clones the current working directory's
EEPROM for each trial and reuses the real-AirSim flight runner. Stop other
SITL/RC senders, start UE Play on a valid ground surface, disable API
control and leave at least 8 m of clear space around the launch point:

    python3 src/test/sitl/airsim_yaw_tune.py \
        --binary obj/main/betaflight_SITL.elf --eeprom eeprom.bin --vehicle Copter --apply

Without `--apply` it does not change your EEPROM. The bounded search varies
Yaw P/I/F and Yaw feedforward-hold gain, not roll/pitch, D or filters. Each
trial takes off, applies 1.25 s opposite Yaw pulses followed by 8 s neutral
observation windows, and lands before changing gains. It compares AirSim
body Yaw rates with the firmware's measured setpoint (MSP FEEDFORWARD debug),
including tracking error, response gain, reversal angle and settling time.
Weak/no-turn responses are rejected, not rewarded for their lack of ringing.

Applying requires an independent repeat with settling <=2 s, final Yaw-rate
RMS <=1.5 deg/s, reversal <=1.5 deg, acceptable tracking and at least 10%
score improvement over a repeated baseline. Otherwise the original remains
unchanged and the command fails explicitly. Original EEPROM, trajectories,
per-pulse CSVs, metrics and the final report are kept in a new artifact
directory (`--output`). A validated result also produces `recommended.cfg`.
The script refuses to overwrite an EEPROM changed by another process.

Once ringing is controlled, `--objective response` additionally scores the
entire pulse's tracking error and the sustained time to reach 80% of the
requested rate. This prevents a slow but quiet response from winning:

    python3 src/test/sitl/airsim_yaw_tune.py \
        --binary obj/main/betaflight_SITL.elf --eeprom eeprom.bin \
        --vehicle Copter --objective response --apply

This search leaves I unchanged, tries P up to 150, F up to 120 and
feedforward-hold gain up to 30. The response candidate must still pass all
ring-down limits, reach 80% in <=0.6 s, track steady rate within 10%, and
keep peak overshoot <=15%. The independently repeated response score and
rise time must each improve by at least 10% before applying. These bounds
are simulation-specific and do not certify hardware or aggressive maneuvers.

Offline scoring/persistence checks:

    PYTHONPATH=src/test/sitl python3 -m unittest airsim_yaw_tune_unittest

### Sixth-detent GPS Rescue / one-switch return home

The sixth AUX1 detent (2000 us) selects permanent box id 46, GPS Rescue.
Use `src/test/sitl/airsim_rescue_test.py` with UE Play, GPS ready, API control
disabled and no other RC sender:

    python3 src/test/sitl/airsim_rescue_test.py \
        --binary obj/main/betaflight_SITL.elf --eeprom eeprom.bin --vehicle Copter

The test clones the source EEPROM, takes off, flies >30 m using ordinary
pilot RC in POSHOLD+ALTHOLD, brakes, then changes only the mode dial to
GPSRESCUE with neutral sticks. It verifies actual return near the arming
point, a fresh ground contact, automatic disarm and no unintended re-arm.
No outbound mission, OFFBOARD controller or synthetic GPS/pose is used.
The flight corridor must be clear for about 40 m and up to 12 m high.

The temporary simulation profile uses FIXED_ALT=8 m, 2.5 m/s return speed,
1.5 m/s ascent and 0.5 m/s descent. The test never changes the source EEPROM.
To explicitly persist this simulation-only profile after reviewing it:

    python3 src/test/sitl/airsim_rescue_test.py --print-config > rescue.cfg
    obj/main/betaflight_SITL.elf --config rescue.cfg

Restart SITL from the same working directory. Keep the mode dial out of
Rescue when arming, wait for GPS fix/home, and keep ARM high during return.
With ENABLE_RESCUE_PLAN (default on this SITL), the GPS Rescue switch invokes
a synthesized climb/home/land mission; MSP shows AUTOPILOT+ALTHOLD+POSHOLD
instead of the legacy GPS_RESCUE flight-mode bit. Loss of GPS/home can cause
degraded landing rather than return. The 8 m profile is not for real aircraft.
