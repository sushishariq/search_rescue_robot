#!/bin/bash
export TURTLEBOT3_MODEL=waffle
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
[ -f /sar_ws/install/setup.bash ] && source /sar_ws/install/setup.bash

# When Docker runs this file at startup, hand over to the given command (e.g. bash).
# When .bashrc sources it (extra shells), just set variables and stop.
[ "${BASH_SOURCE[0]}" = "$0" ] && exec "$@"
