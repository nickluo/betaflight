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
ALTHOLD / POSHOLD+ALTHOLD / reserved at 1000/1200/1400/1600/1800/2000 us.
The fifth detent holds both horizontal position and altitude. ARM follows
the switch level (button value 1 -> 2000 us, value 0 -> 1000 us), including
initial device-state events; it no longer toggles on each press. AUX2 high
requests OFFBOARD; AUX4 remains an unbound Trigger.
For AirSim, also enable GPS with the VIRTUAL provider, set `trust_mag = ON`
and use the appropriate `ap_hover_throttle` (1590 for the default 1 kg frame).
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

The script provisions and launches its own SITL in a new artifact directory;
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

Artifacts include `config.txt`, `provision.log`, `sitl.log`,
`trajectory.csv` (AirSim and FC attitude, position, velocity, modes and
arming flags), and `report.json`. Use `--output <new-directory>` to select
their location.

`--hover-pwm` defaults to 1590 for AirSim's 1 kg BetaFlight QuadX with four
4.18 N rotors (about 59% collective), not the harness plant's 30% hover
thrust. Different mass/rotors require a matching value. The test enforces
a 15 m height / 45 m horizontal / 45 degree tilt envelope. These are
simulation-only tests, not hardware flight procedures.

Offline runner checks:

    python3 -m unittest discover -s src/test/sitl -p airsim_control_test_unittest.py
