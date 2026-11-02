#!/bin/bash
# Entrypoint of all images: sources ROS, the workspace built into the image (/ws/install, robot images)
# and the workspace mounted from the host ($KPI_WS/install, if it is built), then runs the command.
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
[ -n "${KPI_WS:-}" ] && [ -f "${KPI_WS}/install/setup.bash" ] && source "${KPI_WS}/install/setup.bash"

# Devices are bind-mounted with /dev, so the container starts even if a device is missing (e.g. the USB
# lidar is not enumerated yet at boot). Wait for them here; on timeout exit, and the restart policy retries.
if [ -n "${KPI_WAIT_DEVICES:-}" ]; then
  timeout=${KPI_WAIT_TIMEOUT:-60}
  for dev in $KPI_WAIT_DEVICES; do
    i=0
    while [ ! -e "$dev" ] && [ "$i" -lt "$timeout" ]; do sleep 1; i=$((i + 1)); done
    if [ ! -e "$dev" ]; then
      echo "kpi-entrypoint: device $dev not found after ${timeout}s" >&2
      exit 1
    fi
  done
fi

exec "$@"
