#!/usr/bin/env bash
# Sources ROS + workspace overlays, then exec's whatever was passed to the
# container (defaults to bash). Keeps `docker run ... ros2 launch ...` working
# without having to wrap every invocation in `bash -lc`.
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
exec "$@"
