# drone_ws

A quadcopter simulated in Gazebo Sim and flown from ROS 2.

## Environment

ROS 2 **Jazzy** and Gazebo Sim **8.10** are installed as conda packages
(RoboStack) in a micromamba environment named `ros2` — *not* via apt. Nothing
is on the system `PATH`, so every command goes through the environment:

```bash
export MAMBA_ROOT_PREFIX=$HOME/micromamba
micromamba activate ros2        # or: micromamba run -n ros2 <command>
```

## Build

```bash
cd ~/drone_ws
colcon build
source install/setup.bash
```

Do **not** pass `--symlink-install`: it runs `setup.py develop`, which
setuptools 84 no longer provides. If a build ever fails with
`option --uninstall not recognized`, delete `build/ install/ log/` and rebuild.

## Run

```bash
ros2 launch drone_bringup drone_sim.launch.py                 # Gazebo GUI
ros2 launch drone_bringup drone_sim.launch.py headless:=true  # server only
ros2 launch drone_bringup drone_sim.launch.py takeoff:=true altitude:=4.0
ros2 launch drone_bringup drone_sim.launch.py mission:=true b_x:=6.0 b_y:=-2.0
```

One launch brings up the world, the ROS 2 <-> Gazebo bridge, and (optionally)
the takeoff node or the A-to-B mission.

| argument   | default           | meaning                                    |
|------------|-------------------|--------------------------------------------|
| `world`    | `drone_world.sdf` | world file in `drone_bringup/worlds`       |
| `headless` | `false`           | run the server without the GUI             |
| `takeoff`  | `false`           | arm and climb once the sim is up           |
| `altitude` | `3.0`             | target altitude in metres above the launch point |
| `mission`  | `false`           | take off at A, cruise to B, land there     |
| `a_x` `a_y`| `0.0` `0.0`       | point A, for `mission:=true`               |
| `b_x` `b_y`| `5.0` `4.0`       | point B, for `mission:=true`               |

The launch file sets `GZ_RENDERING_PLUGIN_PATH` and
`GZ_RENDERING_RESOURCE_PATH` from `CONDA_PREFIX` before starting Gazebo. The
RoboStack conda build bakes in an install prefix that does not survive
relocation, so without them the GUI fails with `Failed to load plugin
[gz-rendering-ogre2]`, then cannot find its shader media. Rendering falls back
to Mesa software GL on this machine (no GPU), so the GUI is usable but slow.

## Keyboard control

Two terminal tabs. `scripts/` wraps the micromamba invocation so neither
command is a mouthful:

```bash
~/drone_ws/scripts/sim.sh     # tab 1: the simulator, leave it running
~/drone_ws/scripts/fly.sh     # tab 2: the keyboard
```

Both must be real terminal tabs. `teleop_key` reads single keypresses, so
anything that detaches stdin - a pipe, a launch file, an editor task, or
Claude Code's `!` prefix - makes it exit immediately with a message saying so.

| key                  | does                                   |
|----------------------|----------------------------------------|
| up / down arrow      | climb / descend                        |
| left / right arrow   | slide left / right                     |
| `w` / `s`            | forward / back                         |
| `a` / `d`            | yaw left / right                       |
| space                | stop and hover here                    |
| `t`                  | take off to `altitude` (default 3 m)   |
| `l`                  | land straight down, then disarm        |
| `e`                  | toggle the motors                      |
| `+` / `-`            | speed scale up / down                  |
| `?`                  | print the keys                         |
| `q`                  | quit, disarming first                  |

The first movement key arms the motors, so you can just fly. The node
re-publishes the arm state at 2 Hz rather than once: a ROS 2 publisher discards
whatever it sends before its subscriber has been matched, and a single lost
`enable` leaves `MulticopterVelocityControl` ignoring every `cmd_vel` that
follows, with nothing on any topic to explain why. The symptom is that the
keyboard looks completely dead while `/quadcopter/cmd_vel` is plainly being
published - the giveaway is the bridge log, which shows a `Twist` forwarded but
no `Bool`. Movement keys are
*held*, not latched: a terminal reports keypresses but never key releases, so
each command outlives its last keypress by `hold_time` (0.4 s) and then decays
to a hover. Keyboard auto-repeat keeps it alive while you hold the key down.

The one-shot keys (`t`, `l`, `e`, `space`, `+`, `-`) behave differently from
the movement keys, because a terminal auto-repeats *any* key you hold down.
Movement keys want that. The others do not: holding `l` used to reset the
touchdown detector on every repeat, so the drone descended for ever and never
reported landing, and holding `e` flapped the motors on and off many times a
second. Entering a mode you are already in is now a no-op, landing while
disarmed is a no-op, and the rest are debounced by `hold_time`.

This node is not started from the launch file: launch owns the stdin of every
process it spawns, and reading single keypresses needs a terminal of its own.
It puts stdin in *cbreak* mode rather than raw mode, which leaves Ctrl-C and
newline translation working, and restores the terminal on any exit.

## Point A to point B

```bash
ros2 run drone_bringup mission --ros-args \
    -p b_x:=6.0 -p b_y:=-2.0 -p cruise_altitude:=4.0
```

Arm, climb where it stands, fly to A if it is not already there, cruise to B,
then descend onto B and disarm. Points are `(x, y)` in the world frame - the
frame `/odom` reports - and `cruise_altitude` is above the launch point.

Two things this node has to get right:

**`cmd_vel` is a body-frame velocity.** Upstream's own demo world says so: the
multicopter controller takes "linear velocity and yaw angular velocity in the
body frame of the vehicle". `/odom` is in the world frame, so every horizontal
command is the world-frame error rotated by the current yaw
(`flight.world_to_body`). Skip that and the drone flies at an angle to its
target - or in a circle, if it is yawing.

**The ground is not where the model spawns.** `base_link` spawns at z=0.2 but
comes to rest at z=0.04, sitting on its own 0.08 m collision box. Rather than
hard-code either number, `flight.Touchdown` watches the altitude during a
commanded descent and calls it down once `z` stops falling for 0.8 s. At the
slowest rate the descent ever commands, 0.25 m/s, the drone covers 0.2 m in
that window against a 0.02 m threshold, so a real descent cannot look like a
touchdown.

## Tests

```bash
# Unit tests for the frame and touchdown helpers - no simulator needed.
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/drone_bringup/test/test_flight.py

# The keyboard, flown for real: needs the sim already up.
ros2 launch drone_bringup drone_sim.launch.py headless:=true &
python3 src/drone_bringup/test/teleop_flight_check.py
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` is not optional in this environment: the
`launch_testing` and `launch_testing_ros` pytest plugins that ship in the
RoboStack env declare hooks the installed pytest no longer recognises, so
pytest dies during collection before it runs anything. Disabling entry-point
plugins sidesteps both.

`teleop_flight_check.py` drives the node through a pty, because reading single
keypresses needs a terminal and a pipe will not do. It presses `t`, every
arrow, `w`, then `l` and `q`, and checks `/odom` moved the way each key
promises - including that the drone settles back to a hover once the keys stop.

## Topics

| topic                  | type                      | direction   |
|------------------------|---------------------------|-------------|
| `/quadcopter/enable`   | `std_msgs/Bool`           | ROS -> Gazebo |
| `/quadcopter/cmd_vel`  | `geometry_msgs/Twist`     | ROS -> Gazebo |
| `/odom`                | `nav_msgs/Odometry`       | Gazebo -> ROS |
| `/imu`                 | `sensor_msgs/Imu`         | Gazebo -> ROS |

The controller ignores `cmd_vel` until `true` is published on
`/quadcopter/enable`. Flying it by hand:

```bash
ros2 topic pub -1 /quadcopter/enable std_msgs/msg/Bool '{data: true}'
ros2 topic pub -1 /quadcopter/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 1.5}, angular: {z: 0.0}}'
ros2 topic echo /odom --once
```

## The model

`drone_bringup/models/quadcopter/model.sdf` is hand-written: a 1.5 kg
`base_link` with four rotor links on revolute joints, a
`MulticopterMotorModel` plugin per rotor, `MulticopterVelocityControl` as the
flight controller, plus odometry, IMU and air-pressure sensors.

All three flight nodes measure `altitude` from wherever the drone is when it
arms, not from z=0 - the model spawns 0.16 m above where it comes to rest, so
an absolute target would quietly be off by that much, and one launch argument
feeds both `takeoff` and `mission`.

`drone_bringup/takeoff.py` arms the controller and tracks the target altitude
with a proportional climb rate. A constant climb rate cut to zero on arrival
overshoots by about a metre, because the controller zeroes the drone's
*velocity* only after it has already carried past the setpoint. `mission.py`
and `teleop_key.py` taper the same way, and share the frame and touchdown
helpers in `drone_bringup/flight.py`.

## Verified

All headless, on this machine.

`takeoff` with `altitude:=4.0`, from rest at z=0.039999: held **4.040039 m**
(rest + 4.000) with lateral position at ~2e-6 m.

`mission` to B=(5, 4) at 3 m: climbed in ~3 s, cruised in ~5.5 s, landed at
**(5.00, 4.00, 0.04)** - 0.00 m from B.

`mission` to B=(3, -3) from a deliberately yawed start - the drone was spun to
**132 deg** (quaternion z=0.915, w=0.403) and left hovering before the node
started: landed at **(3.00, -3.00, 0.04)**, 0.00 m from B. This is the test
that actually exercises `world_to_body`; with the rotation missing or
backwards, the drone would set off 132 deg away from B.

`scripts/fly.sh`, driven through a pty against a live sim from `scripts/sim.sh`:
`t` pressed with no warm-up armed the drone and took it to 3.172 m, the right
arrow moved it to y=-4.74, `q` exited 0, and the bridge forwarded both the
`Twist` and the `Bool`.

`teleop_key`, driven through a pty against a live sim: all 17 checks pass,
including `t` pressed on the very first tick and `l` held down throughout the
descent -
`t` climbed and held 3 m, the right arrow moved it to y=-4.7, `w` moved it to
x=4.8, the up and down arrows took it 2.8 -> 6.2 -> 2.9 m, position drift after
releasing the keys measured 0.000 m, and `l` landed it at z=0.040 and disarmed.
`q` exited with status 0 and no traceback. Pressing `l` while still flying
forward carried 0.16 m horizontally - just the momentum - rather than riding
the stale command all the way down.

A note on testing this by hand: killing the `ros2 launch` process leaves the
Gazebo server and the bridge orphaned, and a second sim started alongside them
shares the same transport, so `/odom` then carries two drones' poses and any
reading you take is worthless. Start it with `setsid` and kill the process
group (`kill -TERM -- -$PID`), then check nothing survived.
