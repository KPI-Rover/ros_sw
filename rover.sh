#!/bin/bash
# One entry point for working with the simulation on the PC and with your robot. Run ./rover.sh help
set -euo pipefail

REPO="$(cd "$(dirname "$0")" && pwd)"
cd "$REPO"

usage() {
  cat <<'EOF'
Usage: ./rover.sh <command> [args]

PC
  pc [command...]      open the PC container (zellij), or run a command in it
  pc-update            download the latest PC image
  deps                 import external packages from deps.repos into src/external/

Robot (the address is saved by `setup` in .rover, or set ROVER_HOST=user@host)
  setup <user@host> [--lidar rplidar|coin] [--wheel-radius M] [--domain-id N]
                       prepare a new RPi and start the robot software (safe to repeat)
  status               what is running on the robot and from which commit
  update               download the current image tag and restart
  use <tag>            switch to another image: main, a branch name, sha-<commit> or local
  restart              start the robot software again (e.g. after `overlay`)
  logs                 follow the robot software logs
  overlay <pkg>...     copy packages to the robot and open a shell there to build and run them
  overlay --clean      remove the overlay from the robot
  rviz                 RViz on the PC, connected to the robot
  wifi                 list the Wi-Fi networks saved on the robot
  wifi <ssid>          add a Wi-Fi network to the robot (asks for the password), the old ones are kept
  wifi --remove <name> remove a saved Wi-Fi network
  build                build the robot image on the robot from local sources (no CI), tag `local`
  doctor               collect diagnostics to paste into an issue
  ssh [command...]     ssh to the robot
EOF
}

die() { echo "ERROR: $*" >&2; exit 1; }

# ---------- robot connection ----------

rover_host() {
  if [ -z "${ROVER_HOST:-}" ] && [ -f .rover ]; then
    ROVER_HOST=$(sed -n 's/^ROVER_HOST=//p' .rover)
  fi
  [ -n "${ROVER_HOST:-}" ] || die "robot address is not set: run ./rover.sh setup user@host, or set ROVER_HOST"
  echo "$ROVER_HOST"
}

# Commands on the robot run in ~/kpi-rover, where scripts/rpi-setup/docker-compose.yaml and .env are installed
on_robot() { ssh "$(rover_host)" "cd ~/kpi-rover && $*"; }
on_robot_tty() { ssh -t "$(rover_host)" "cd ~/kpi-rover && $*"; }
robot_env() { on_robot "sed -n 's/^$1=//p' .env"; }

# ---------- PC ----------

pc_env() {
  docker compose version > /dev/null 2>&1 ||
    die "Docker Compose v2 is not installed (\`docker compose\` does not work). On Ubuntu with docker.io: sudo apt install docker-compose-v2; with Docker CE: sudo apt install docker-compose-plugin"
  export HOST_UID HOST_GID VIDEO_GID RENDER_GID
  HOST_UID=$(id -u)
  HOST_GID=$(id -g)
  VIDEO_GID=$(getent group video | cut -d: -f3 || true)
  RENDER_GID=$(getent group render | cut -d: -f3 || true)
  [ -n "$VIDEO_GID" ] || VIDEO_GID=44
  [ -n "$RENDER_GID" ] || RENDER_GID=$VIDEO_GID
  if [ -n "${DISPLAY:-}" ] && command -v xhost > /dev/null; then
    xhost +local:docker > /dev/null
  fi
  PC_SERVICE=${PC_SERVICE:-pc}
  # WSL2 exposes the Windows GPU as /dev/dxg (any vendor), rendering goes through Mesa's d3d12 driver
  if [ "$PC_SERVICE" = pc ] && [ -e /dev/dxg ]; then
    PC_SERVICE=pc-wsl
  elif [ "$PC_SERVICE" = pc ] && command -v nvidia-container-runtime > /dev/null && docker info --format "{{json .Runtimes}}" 2> /dev/null | grep -q nvidia; then
    PC_SERVICE=pc-nvidia
  fi
}

pc() {
  pc_env
  docker compose run --rm "$PC_SERVICE" "$@"
}

# ---------- commands ----------

cmd_pc_update() {
  pc_env
  docker compose pull "$PC_SERVICE"
}

cmd_deps() {
  pc_env
  docker compose run --rm -T "$PC_SERVICE" vcs import --input deps.repos src < /dev/null
}

cmd_setup() {
  [ $# -ge 1 ] || die "usage: ./rover.sh setup user@host [--lidar rplidar|coin] [--wheel-radius M] [--domain-id N]"
  ROVER_HOST=$1; shift
  ssh -o ConnectTimeout=10 "$ROVER_HOST" true || die "cannot ssh to $ROVER_HOST"
  echo "ROVER_HOST=$ROVER_HOST" > .rover
  ssh "$ROVER_HOST" "mkdir -p ~/kpi-rover"
  scp -q scripts/rpi-setup/setup.sh scripts/rpi-setup/docker-compose.yaml scripts/rpi-setup/99-kpi-rover.rules "$ROVER_HOST:kpi-rover/"
  ssh -t "$ROVER_HOST" "bash ~/kpi-rover/setup.sh $(printf '%q ' "$@")"
}

cmd_status() {
  on_robot "docker compose ps -a robot; echo; grep -E '^(TAG|LIDAR_MODEL|WHEEL_RADIUS|ROS_DOMAIN_ID)=' .env; echo; \
    docker image inspect \"\$(docker compose config --images robot)\" \
      --format 'image:    {{index .RepoTags 0}}
commit:   {{index .Config.Labels \"org.opencontainers.image.revision\"}}
built:    {{index .Config.Labels \"org.opencontainers.image.created\"}}' 2> /dev/null || echo 'image is not downloaded'"
}

cmd_update() {
  if [ "$(robot_env TAG)" != local ]; then
    on_robot "docker compose pull robot"
  fi
  on_robot "docker compose up -d robot"
}

cmd_use() {
  [ $# -eq 1 ] || die "usage: ./rover.sh use <tag>"
  # Branch names become tags with '/' replaced, like in CI
  local tag=${1//\//-}
  on_robot "sed -i '/^TAG=/d' .env && echo 'TAG=$tag' >> .env"
  cmd_update
}

cmd_restart() { on_robot "docker compose up -d robot"; }

cmd_logs() { on_robot_tty "docker compose logs -f --tail=200 robot"; }

# Finds a package directory by its name (or accepts a path to it)
package_dir() {
  if [ -f "$1/package.xml" ]; then
    echo "${1%/}"
    return
  fi
  local xml
  xml=$(grep -rl --include=package.xml "<name>$1</name>" src 2> /dev/null | head -n 1 || true)
  [ -n "$xml" ] || die "package $1 not found (run ./rover.sh deps for external packages)"
  dirname "$xml"
}

cmd_overlay() {
  [ $# -ge 1 ] || die "usage: ./rover.sh overlay <package>... | --clean"
  local host
  host=$(rover_host)
  if [ "$1" = --clean ]; then
    on_robot "rm -rf overlay/src/* overlay/build overlay/install overlay/log"
    echo "Overlay removed. The robot software uses only the image now: ./rover.sh restart"
    return
  fi
  local pkg dir
  for pkg in "$@"; do
    dir=$(package_dir "$pkg")
    rsync -a --delete --exclude .git "$dir/" "$host:kpi-rover/overlay/src/$(basename "$dir")/"
    echo "Copied $dir"
  done
  if [ "$(robot_env TAG)" != local ]; then
    on_robot "docker compose pull -q robot"
  fi
  on_robot "docker compose stop robot"
  cat <<'EOF'

Inside the container:
  colcon build --symlink-install && source install/setup.bash
  ros2 launch kpi_rover_bringup robot.launch.py
The robot software is stopped while you work here; start it again with ./rover.sh restart

EOF
  on_robot_tty "docker compose run --rm dev"
}

cmd_rviz() {
  local domain ip
  domain=$(robot_env ROS_DOMAIN_ID)
  # Static peer in addition to multicast discovery: some home routers drop multicast
  ip=$(on_robot "hostname -I" | awk '{print $1}')
  [ -f install/setup.bash ] || die "the workspace is not built: ./rover.sh pc colcon build"
  ROS_DOMAIN_ID=${domain:-0} ROS_STATIC_PEERS=$ip pc ros2 launch kpi_rover_bringup rviz.launch.py
}

# Wi-Fi networks of the robot are NetworkManager connections (Raspberry Pi OS). The password goes to the robot
# only through stdin, so it does not appear in process lists or in sudo logs.
cmd_wifi() {
  local host
  host=$(rover_host)
  case "${1:-}" in
    "")
      ssh "$host" "nmcli -f NAME,TYPE,AUTOCONNECT,DEVICE connection show | awk 'NR == 1 || /wifi|wireless/'"
      ;;
    --remove)
      [ $# -eq 2 ] || die "usage: ./rover.sh wifi --remove <name>"
      ssh "$host" "sudo nmcli connection delete id $(printf '%q' "$2")"
      ;;
    *)
      [ $# -eq 1 ] || die "usage: ./rover.sh wifi <ssid>"
      local ssid=$1 password file
      read -rsp "Password of $ssid: " password
      echo
      [ ${#password} -ge 8 ] && [ ${#password} -le 63 ] || die "a WPA password has 8..63 characters"
      file="kpi-wifi-$(printf '%s' "$ssid" | tr -c 'A-Za-z0-9_-' '_').nmconnection"
      printf '[connection]\nid=%s\nuuid=%s\ntype=wifi\nautoconnect=true\n\n[wifi]\nmode=infrastructure\nssid=%s\n\n[wifi-security]\nkey-mgmt=wpa-psk\npsk=%s\n\n[ipv4]\nmethod=auto\n\n[ipv6]\nmethod=auto\n' \
        "$ssid" "$(cat /proc/sys/kernel/random/uuid)" "$ssid" "$password" |
        ssh "$host" "sudo install -m 600 /dev/stdin /etc/NetworkManager/system-connections/$file && sudo nmcli connection reload"
      echo "Saved. The robot connects to $ssid when it is in range (other saved networks are kept)."
      ;;
  esac
}

cmd_build() {
  local host revision
  host=$(rover_host)
  revision=$(git rev-parse HEAD)$(git diff --quiet HEAD || echo "-dirty")
  DOCKER_HOST="ssh://$host" docker buildx build --builder default -f docker/Dockerfile --target robot \
    --label "org.opencontainers.image.revision=$revision" \
    --label "org.opencontainers.image.created=$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
    -t ghcr.io/kpi-rover/robot:local .
  cmd_use local
}

cmd_doctor() {
  echo '```'
  echo "## PC"
  echo "repo:     $(git rev-parse --abbrev-ref HEAD) $(git rev-parse --short HEAD)$(git diff --quiet HEAD || echo ' (uncommitted changes)')"
  echo "os:       $(. /etc/os-release && echo "$PRETTY_NAME")"
  echo "docker:   $(docker version --format '{{.Server.Version}}' 2>&1)"
  echo "compose:  $(docker compose version --short 2>&1)"
  [ -e /dev/dxg ] && echo "wsl:      $(uname -r), WSLg: $([ -d /mnt/wslg ] && echo yes || echo no)"
  echo "groups:   video=$(getent group video | cut -d: -f3) render=$(getent group render | cut -d: -f3)"
  command -v nvidia-smi > /dev/null && echo "gpu:      $(nvidia-smi --query-gpu=name --format=csv,noheader 2>&1)"
  if [ -n "${DISPLAY:-}" ]; then
    echo "renderer: $(pc glxinfo -B 2>&1 | sed -n 's/.*OpenGL renderer string: //p' | tr -d '\r')"
  fi
  if [ -n "${ROVER_HOST:-}" ] || [ -f .rover ]; then
    echo
    echo "## Robot $(rover_host)"
    on_robot "
      echo \"model:    \$(tr -d '\0' < /proc/device-tree/model)\"
      echo \"os:       \$(. /etc/os-release && echo \"\$PRETTY_NAME\") \$(uname -m)\"
      echo \"uptime:  \$(uptime)\"
      echo \"power:    \$(vcgencmd get_throttled 2>&1) (0x0 is OK)\"
      echo \"disk:     \$(df -h / | awk 'NR==2 {print \$4\" free of \"\$2}')\"
      echo; echo '### settings'; cat .env
      echo; echo '### devices'; ls -l /dev/ttyAMA* /dev/ttyUSB* /dev/lidar /dev/sc_mini 2>&1
      echo; echo '### boot config'; grep -E 'uart|^\[' /boot/firmware/config.txt; grep -o 'console=[^ ]*' /boot/firmware/cmdline.txt
      echo; echo '### container'; docker compose ps -a robot 2>&1
      docker image inspect \"\$(docker compose config --images robot)\" --format 'commit: {{index .Config.Labels \"org.opencontainers.image.revision\"}}' 2>&1
      echo; echo '### last logs'; docker compose logs --no-color --tail=40 robot 2>&1
    " 2>&1 || echo "cannot reach the robot"
  fi
  echo '```'
}

# ---------- main ----------

command=${1:-help}
shift || true
case "$command" in
  pc) pc "$@" ;;
  pc-update) cmd_pc_update ;;
  deps) cmd_deps ;;
  setup) cmd_setup "$@" ;;
  status) cmd_status ;;
  update) cmd_update ;;
  use) cmd_use "$@" ;;
  restart) cmd_restart ;;
  logs) cmd_logs ;;
  overlay) cmd_overlay "$@" ;;
  rviz) cmd_rviz ;;
  wifi) cmd_wifi "$@" ;;
  build) cmd_build ;;
  doctor) cmd_doctor ;;
  ssh) ssh -t "$(rover_host)" "$@" ;;
  help | -h | --help) usage ;;
  *) usage; exit 1 ;;
esac
