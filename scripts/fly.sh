#!/usr/bin/env bash
# Fly the drone from the keyboard. Run this in its own terminal tab, with
# sim.sh already running in another one.
#
#   ~/drone_ws/scripts/fly.sh
#
# It must run in a real terminal: it reads single keypresses, which a pipe or a
# launch file cannot deliver.
export MAMBA_ROOT_PREFIX="$HOME/micromamba"
cd "$HOME/drone_ws" || exit 1
exec "$HOME/.local/bin/micromamba" run -a "" -n ros2 bash -c \
  'source install/setup.bash && exec ros2 run drone_bringup teleop_key'
