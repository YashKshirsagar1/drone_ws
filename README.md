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
```

One launch brings up the world, the ROS 2 <-> Gazebo bridge, and (optionally)
the takeoff node.

| argument   | default           | meaning                                |
|------------|-------------------|----------------------------------------|
| `world`    | `drone_world.sdf` | world file in `drone_bringup/worlds`   |
| `headless` | `false`           | run the server without the GUI         |
| `takeoff`  | `false`           | arm and climb once the sim is up       |
| `altitude` | `3.0`             | target altitude in metres              |

The launch file sets `GZ_RENDERING_PLUGIN_PATH` and
`GZ_RENDERING_RESOURCE_PATH` from `CONDA_PREFIX` before starting Gazebo. The
RoboStack conda build bakes in an install prefix that does not survive
relocation, so without them the GUI fails with `Failed to load plugin
[gz-rendering-ogre2]`, then cannot find its shader media. Rendering falls back
to Mesa software GL on this machine (no GPU), so the GUI is usable but slow.

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

`drone_bringup/takeoff.py` arms the controller and tracks the target altitude
with a proportional climb rate. A constant climb rate cut to zero on arrival
overshoots by about a metre, because the controller zeroes the drone's
*velocity* only after it has already carried past the setpoint.

## Verified

Commanded to 4.0 m, headless: reached the setpoint in ~6 s and held
4.000038 m with lateral position at ~2e-5 m.
