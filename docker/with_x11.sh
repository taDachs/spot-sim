#!/bin/bash
# Run a command against the host's X server (X11 or XWayland).
#
# The host's Xauthority cookie is bound to the host's hostname, which differs
# inside the container. Re-write it as a wildcard cookie so it matches any
# host, without having to open the display with `xhost +`.
set -e
if [ -s /tmp/.host.xauth ]; then
  touch /tmp/.xauth
  xauth -f /tmp/.host.xauth nlist "${DISPLAY}" | sed -e 's/^..../ffff/' | xauth -q -f /tmp/.xauth nmerge -
  export XAUTHORITY=/tmp/.xauth
fi
exec "$@"
