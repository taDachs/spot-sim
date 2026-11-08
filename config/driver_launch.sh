#!/usr/bin/env bash
# Starts the unmodified spot_ros2 driver with config/driver.yaml, applying
# settings from the environment:
#   RGB_CAMERAS=true|false  the driver's rgb_cameras parameter: request color
#                           (true) or greyscale (false) body camera images
#   DRIVER_ARGS             extra spot_driver.launch.py arguments, e.g.
#                           "depth_registered_mode:=from_spot"
# Arguments are passed on to spot_driver.launch.py.
set -euo pipefail

case "${RGB_CAMERAS:-true}" in
  true | True | 1) rgb_cameras=True ;;
  false | False | 0) rgb_cameras=False ;;
  *) echo "RGB_CAMERAS must be true or false, got '${RGB_CAMERAS}'" >&2; exit 1 ;;
esac

config=/tmp/driver.yaml
sed -E "s/^(\s*rgb_cameras:).*/\1 ${rgb_cameras}/" /config/driver.yaml > "${config}"
grep -q "rgb_cameras: ${rgb_cameras}" "${config}"

# shellcheck disable=SC2086  # DRIVER_ARGS is a list of arguments
exec ros2 launch spot_driver spot_driver.launch.py config_file:="${config}" "$@" ${DRIVER_ARGS:-}
