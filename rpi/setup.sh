#!/bin/bash
# Prepares a Raspberry Pi 5 for the robot software and starts it. Runs ON the RPi: `./rover.sh setup`
# on the PC copies this directory to ~/kpi-rover and runs it. Safe to run again, e.g. to change settings.
# Usage: setup.sh [--lidar rplidar|coin] [--wheel-radius METERS] [--domain-id N]
set -euo pipefail

cd "$(dirname "$0")"

LIDAR_MODEL=""
WHEEL_RADIUS=""
DOMAIN_ID=""
while [ $# -gt 0 ]; do
  case "$1" in
    --lidar) LIDAR_MODEL="$2"; shift 2 ;;
    --wheel-radius) WHEEL_RADIUS="$2"; shift 2 ;;
    --domain-id) DOMAIN_ID="$2"; shift 2 ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
done

BOOT=/boot/firmware
REBOOT_NEEDED=false
step() { echo; echo "==> $*"; }
fail() { echo "ERROR: $*" >&2; exit 1; }

step "Checking the system"
[ "$(uname -m)" = aarch64 ] || fail "64-bit OS is required (uname -m: $(uname -m)). Flash Raspberry Pi OS (64-bit)."
MODEL=$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || true)
echo "$MODEL"
[[ "$MODEL" == *"Raspberry Pi 5"* ]] || echo "WARNING: not a Raspberry Pi 5, the UART setup below is for Pi 5."
[ -f "$BOOT/config.txt" ] || fail "$BOOT/config.txt not found"
sudo -v || fail "sudo is required"

step "Bootloader update (https://github.com/raspberrypi/rpi-eeprom/issues/514)"
sudo apt-get update -qq
sudo apt-get install -y -qq rpi-eeprom > /dev/null
if sudo rpi-eeprom-update -a | tee /dev/stderr | grep -qi reboot; then
  REBOOT_NEEDED=true
fi

step "UART: uart0 and uart2 enabled, no serial console"
OVERLAYS=""
for overlay in uart0-pi5 uart2-pi5; do
  grep -q "^dtoverlay=$overlay\$" "$BOOT/config.txt" || OVERLAYS+="dtoverlay=$overlay"$'\n'
done
if [ -n "$OVERLAYS" ]; then
  printf '\n[pi5]\n%s' "$OVERLAYS" | sudo tee -a "$BOOT/config.txt" > /dev/null
  echo "Added to $BOOT/config.txt:"; echo -n "$OVERLAYS"
  REBOOT_NEEDED=true
fi
if grep -q 'console=serial0,[0-9]*' "$BOOT/cmdline.txt"; then
  sudo sed -i 's/console=serial0,[0-9]* \?//' "$BOOT/cmdline.txt"
  echo "Removed the serial console from $BOOT/cmdline.txt"
  REBOOT_NEEDED=true
fi

step "udev rules for the lidars (/dev/lidar, /dev/sc_mini)"
sudo install -m 644 99-kpi-rover.rules /etc/udev/rules.d/99-kpi-rover.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

step "Docker"
if ! command -v docker > /dev/null; then
  curl -fsSL https://get.docker.com | sudo sh
fi
sudo systemctl enable --now docker > /dev/null
docker compose version > /dev/null 2>&1 || sudo docker compose version > /dev/null || fail "docker compose plugin is missing"
if ! id -nG "$USER" | grep -qw docker; then
  sudo usermod -aG docker "$USER"
  echo "Added $USER to the docker group"
fi

step "Settings (~/kpi-rover/.env)"
touch .env
# Sets KEY to VALUE; with an empty VALUE only adds DEFAULT if KEY is not set yet
set_env() {
  local key=$1 value=$2 default=${3:-}
  if [ -n "$value" ]; then
    sed -i "/^$key=/d" .env && echo "$key=$value" >> .env
  elif ! grep -q "^$key=" .env; then
    echo "$key=$default" >> .env
  fi
}
set_env TAG "" main
set_env LIDAR_MODEL "$LIDAR_MODEL" rplidar
set_env WHEEL_RADIUS "$WHEEL_RADIUS" 0.034
set_env ROS_DOMAIN_ID "$DOMAIN_ID" 0
set_env ECU_DEVICE "" /dev/ttyAMA2
case "$(grep '^LIDAR_MODEL=' .env | cut -d= -f2)" in
  coin) set_env LIDAR_DEVICE /dev/sc_mini ;;
  *) set_env LIDAR_DEVICE /dev/lidar ;;
esac
set_env HOST_UID "$(id -u)"
set_env HOST_GID "$(id -g)"
cat .env
mkdir -p overlay/src

step "Robot software"
sudo docker compose pull robot
sudo docker compose up -d robot

echo
if $REBOOT_NEEDED; then
  echo "Done. A reboot is required for the bootloader/UART changes; the robot software starts after it automatically."
  read -rp "Reboot now? [y/N] " answer
  [[ "$answer" == [yY]* ]] && sudo reboot
else
  echo "Done. The robot software is running and starts on boot."
fi
