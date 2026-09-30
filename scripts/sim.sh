#!/usr/bin/env bash
# Start the simulator. Run this in its own terminal tab and leave it running.
#
#   ~/drone_ws/scripts/sim.sh                 # with the Gazebo window
#   ~/drone_ws/scripts/sim.sh headless:=true  # no window
#
# Any drone_sim.launch.py argument can be passed straight through.
export MAMBA_ROOT_PREFIX="$HOME/micromamba"
cd "$HOME/drone_ws" || exit 1
# -a "" stops micromamba redirecting the streams, so the terminal stays ours.
exec "$HOME/.local/bin/micromamba" run -a "" -n ros2 bash -c \
  'source install/setup.bash && exec ros2 launch drone_bringup drone_sim.launch.py "$@"' \
  drone_sim "$@"
