#!/usr/bin/env bash
# Prepara un Raspberry Pi (Ubuntu 24.04 arm64) para correr tu estrategia en el coche real de Robotek.
#
#   git clone https://github.com/renzodamgo/robotek-learn.git && cd robotek-learn
#   bash tools/setup-pi.sh              # instala y compila (pide sudo cuando hace falta)
#   bash tools/setup-pi.sh --check      # solo comprueba: no instala nada y no mueve nada
#   bash tools/setup-pi.sh --dry-run    # muestra los comandos sin ejecutarlos
#
# Qué hace (se puede repetir: lo que ya está hecho se salta):
#   1. comprueba el sistema (Ubuntu 24.04);
#   2. instala ROS 2 Jazzy base, sin Gazebo ni RViz (tools/install-ubuntu.sh --no-gazebo) y los paquetes apt del coche;
#   3. instala las reglas de udev (tools/99-robotek.rules) y te añade a los grupos dialout, video y gpio;
#   4. compila todo: colcon build --symlink-install.
#
# SEGURIDAD: este script no arranca ningún nodo ni mueve el motor ni el servo. La primera vez que lances el coche
# ponlo sobre soportes (ruedas en el aire). Ver src/robotek_hardware/README.md.
set -euo pipefail

MODE=install
DRY=0
SKIP_BUILD=0
for arg in "$@"; do
  case "$arg" in
    --check) MODE=check ;;
    --dry-run) DRY=1 ;;
    --skip-build) SKIP_BUILD=1 ;;
    -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
    *) echo "Opción desconocida: $arg (usa --help)" >&2; exit 2 ;;
  esac
done

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SUDO=sudo
if [ "$(id -u)" -eq 0 ]; then SUDO=; fi
TARGET_USER="${SUDO_USER:-${USER:-$(id -un)}}"

APT_PACKAGES=(
  git build-essential cmake curl rsync
  ros-jazzy-cv-bridge ros-jazzy-robot-state-publisher ros-jazzy-xacro ros-jazzy-tf2-ros
  python3-serial python3-yaml python3-numpy python3-opencv python3-lgpio v4l-utils
)

say() { echo ">>> $*"; }
warn() { echo "!!! $*" >&2; }
run() {   # ejecuta, o solo muestra el comando con --dry-run
  if [ "$DRY" = 1 ]; then echo "+ $*"; else "$@"; fi
}
have_group() { getent group "$1" >/dev/null 2>&1; }
udev_running() { [ "$DRY" = 1 ] || pgrep -x systemd-udevd >/dev/null 2>&1; }

# ---------------------------------------------------------------------------------------------------------------
check_os() {
  local id="" version="" arch
  if [ -r /etc/os-release ]; then id="$(. /etc/os-release && echo "${ID:-}")"; version="$(. /etc/os-release && echo "${VERSION_ID:-}")"; fi
  arch="$(uname -m)"
  if [ "$id" = ubuntu ] && [ "$version" = 24.04 ]; then
    say "Sistema: Ubuntu 24.04 ($arch)"
    if [ "$arch" != aarch64 ]; then warn "No es arm64 ($arch): funciona, pero el coche real es un Raspberry Pi 5 (aarch64)."; fi
  elif [ "$DRY" = 1 ] || [ "${ROBOTEK_FORCE:-0}" = 1 ]; then
    warn "Sistema ${id:-desconocido} ${version:-?}: esto solo está probado en Ubuntu 24.04. Sigo porque es --dry-run o ROBOTEK_FORCE=1."
  else
    echo "Este script es para Ubuntu 24.04 (el Raspberry Pi del coche). Sistema detectado: ${id:-desconocido} ${version:-?}." >&2
    echo "Para el simulador en tu portátil usa tools/install-ubuntu.sh (o la guía, en macOS)." >&2
    exit 1
  fi
}

# ---------------------------------------------------------------------------------------------------------------
install_ros() {
  if [ -f /opt/ros/jazzy/setup.bash ]; then
    say "ROS 2 Jazzy ya está instalado"
  else
    say "Instalando ROS 2 Jazzy base (sin Gazebo ni RViz)"
    run bash "$REPO/tools/install-ubuntu.sh" --no-gazebo
  fi
}

install_apt() {
  say "Paquetes apt del coche: ${APT_PACKAGES[*]}"
  run $SUDO apt-get update
  run $SUDO apt-get install -y "${APT_PACKAGES[@]}"
}

# ---------------------------------------------------------------------------------------------------------------
install_udev() {
  local rules=/etc/udev/rules.d/99-robotek.rules
  if ! udev_running; then
    warn "systemd-udevd no está corriendo (¿un contenedor?): salto las reglas de udev y los grupos."
    return
  fi
  if [ -f "$rules" ] && cmp -s "$REPO/tools/99-robotek.rules" "$rules"; then
    say "Reglas de udev ya instaladas"
  else
    say "Instalando reglas de udev ($rules)"
    run $SUDO install -m 644 "$REPO/tools/99-robotek.rules" "$rules"
    run $SUDO udevadm control --reload-rules
    run $SUDO udevadm trigger
  fi
  if [ "$(id -u)" -eq 0 ] && [ -z "${SUDO_USER:-}" ]; then
    warn "Corres como root sin SUDO_USER: no sé a qué usuario añadir a los grupos. Hazlo con: usermod -aG dialout,video,gpio USUARIO"
    return
  fi
  if ! have_group gpio; then
    say "Creando el grupo gpio"
    run $SUDO groupadd -f gpio
  fi
  local missing=() group
  for group in dialout video gpio; do
    if ! id -nG "$TARGET_USER" 2>/dev/null | tr ' ' '\n' | grep -qx "$group"; then missing+=("$group"); fi
  done
  if [ ${#missing[@]} -gt 0 ]; then
    local list; list="$(IFS=,; echo "${missing[*]}")"
    say "Añadiendo a $TARGET_USER a los grupos: $list (cierra la sesión y vuelve a entrar para que cuenten)"
    run $SUDO usermod -aG "$list" "$TARGET_USER"
  else
    say "$TARGET_USER ya está en dialout, video y gpio"
  fi
}

# ---------------------------------------------------------------------------------------------------------------
build_workspace() {
  if [ "$SKIP_BUILD" = 1 ]; then say "Salto la compilación (--skip-build)"; return; fi
  if [ ! -d "$REPO/src/my_open" ]; then
    warn "Aún no existe src/my_open (tu estrategia). Compilo el resto; créala como dice el README (ros2 pkg create ... my_open)."
  fi
  say "Compilando: colcon build --symlink-install (varios minutos en un Pi)"
  if [ "$DRY" = 1 ]; then
    echo "+ (cd $REPO && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install)"
    return
  fi
  (
    cd "$REPO"
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    set -u
    colcon build --symlink-install
  )
}

next_steps() {
  cat <<NEXT

>>> Listo. Siguientes pasos (en el Pi, desde $REPO):
    1. Si te añadí a grupos nuevos, cierra la sesión y vuelve a entrar.
    2. bash tools/setup-pi.sh --check          # placa, LiDAR, cámara y grupos
    3. source install/setup.bash
    4. PON EL COCHE SOBRE SOPORTES (ruedas en el aire) la primera vez.
    5. ros2 launch robotek_course car.launch.py strategy:=my_open
       Los LED de la placa se ponen verdes cuando LiDAR, IMU y cámara dan datos; el botón de la placa es el Start.
NEXT
}

# ---------------------------------------------------------------------------------------------------------------
# --check: solo mira, no cambia nada
ok_count=0; bad_count=0
good() { echo "  ok     $*"; ok_count=$((ok_count+1)); }
bad() { echo "  FALTA  $*"; bad_count=$((bad_count+1)); }
note() { echo "  aviso  $*"; }

first_existing() { local p; for p in "$@"; do if [ -e "$p" ]; then echo "$p"; return 0; fi; done; return 1; }

do_check() {
  say "Comprobación (no se instala ni se mueve nada)"
  local id="" version=""
  if [ -r /etc/os-release ]; then id="$(. /etc/os-release && echo "${ID:-}")"; version="$(. /etc/os-release && echo "${VERSION_ID:-}")"; fi
  if [ "$id" = ubuntu ] && [ "$version" = 24.04 ]; then good "Ubuntu 24.04 ($(uname -m))"; else bad "Ubuntu 24.04 (este sistema: ${id:-?} ${version:-?})"; fi
  if [ -f /opt/ros/jazzy/setup.bash ]; then good "ROS 2 Jazzy"; else bad "ROS 2 Jazzy (bash tools/setup-pi.sh)"; fi
  if [ -f "$REPO/install/setup.bash" ]; then
    good "workspace compilado (install/setup.bash)"
    local packages
    packages="$(cd "$REPO" && set +u && source /opt/ros/jazzy/setup.bash 2>/dev/null && source install/setup.bash 2>/dev/null && ros2 pkg list 2>/dev/null || true)"
    local pkg
    for pkg in rrc_lite_msgs rrc_lite_driver ldlidar_stl_ros2 robotek_hardware robotek_course my_open; do
      if grep -qx "$pkg" <<<"$packages"; then good "paquete $pkg"; else bad "paquete $pkg (compila: bash tools/setup-pi.sh)"; fi
    done
  else
    bad "workspace sin compilar (bash tools/setup-pi.sh)"
  fi

  local board lidar camera
  if board="$(first_existing /dev/rrc_lite /dev/ttyACM0)"; then good "placa RRC Lite en $board"; else bad "placa RRC Lite (USB 1a86:55d4, /dev/ttyACM0): enchúfala con el cable USB de datos"; fi
  if lidar="$(first_existing /dev/ldlidar /dev/ttyUSB0)"; then good "LiDAR LD19 en $lidar"; else bad "LiDAR LD19 (USB 10c4:ea60, /dev/ttyUSB0): revisa el cable"; fi
  if camera="$(first_existing /dev/v4l/by-id/*-video-index0 2>/dev/null)"; then good "cámara en $camera"
  elif [ -e /dev/video0 ]; then note "cámara solo como /dev/video0 (en el Pi 5 ese número puede ser de otro aparato): ls /dev/v4l/by-id/"
  else bad "cámara USB (/dev/v4l/by-id/...-video-index0)"; fi
  if [ -e /dev/gpiochip4 ]; then good "GPIO del motor (/dev/gpiochip4)"; else bad "/dev/gpiochip4 (el motor TB6612 es del Pi 5; en otro Pi cambia gpiochip en robot.yaml)"; fi
  if python3 -c 'import lgpio' 2>/dev/null; then good "python3-lgpio"; else bad "python3-lgpio (sudo apt install python3-lgpio)"; fi
  if python3 -c 'import serial' 2>/dev/null; then good "python3-serial"; else bad "python3-serial (sudo apt install python3-serial)"; fi

  local group
  for group in dialout video gpio; do
    if [ "$(id -u)" -eq 0 ]; then note "corres como root: no se comprueban los grupos"; break
    elif id -nG 2>/dev/null | tr ' ' '\n' | grep -qx "$group"; then good "estás en el grupo $group"; else bad "no estás en el grupo $group (sudo usermod -aG $group \$USER y vuelve a entrar)"; fi
  done
  if [ -f /etc/udev/rules.d/99-robotek.rules ]; then good "reglas de udev instaladas"; else note "reglas de udev sin instalar (bash tools/setup-pi.sh)"; fi
  echo
  echo ">>> $ok_count bien, $bad_count por resolver"
  [ "$bad_count" -eq 0 ]
}

# ---------------------------------------------------------------------------------------------------------------
if [ "$MODE" = check ]; then
  do_check
  exit $?
fi

check_os
install_ros
install_apt
install_udev
build_workspace
if [ "$DRY" = 1 ]; then say "Fin del --dry-run: no se ejecutó nada."; else next_steps; fi
