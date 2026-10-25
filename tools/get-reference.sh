#!/usr/bin/env bash
# Instala la estrategia de referencia del reto Open (la terminada, del paso 9) como paquete src/my_open.
#
#   bash tools/get-reference.sh             # la extrae de la etiqueta step-9 y compila solo my_open
#   bash tools/get-reference.sh --force     # si ya tienes src/my_open: copia de seguridad src/my_open.bak-FECHA y lo reemplaza
#   bash tools/get-reference.sh --no-build  # solo la extrae (lo usa tools/setup-pi.sh, que compila todo después)
#
# En la rama main tu paquete src/my_open no existe (es el que escribes tú en la guía). Si ya existe, este script
# NO lo toca (avisa y sale) salvo con --force. Se puede repetir sin riesgo.
set -euo pipefail

TAG=step-9
FORCE=0
BUILD=1
for arg in "$@"; do
  case "$arg" in
    --force) FORCE=1 ;;
    --no-build) BUILD=0 ;;
    -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
    *) echo "Opción desconocida: $arg (usa --help)" >&2; exit 2 ;;
  esac
done

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

if [ -d src/my_open ]; then
  if [ "$FORCE" = 0 ]; then
    echo ">>> Ya existe src/my_open: no lo toco. Usa --force para guardarlo como src/my_open.bak-FECHA y reemplazarlo."
    exit 0
  fi
  backup="src/my_open.bak-$(date +%Y%m%d-%H%M%S)"
  echo ">>> --force: copia de seguridad en $backup"
  mv src/my_open "$backup"
  touch "$backup/COLCON_IGNORE"   # que colcon no vea dos paquetes my_open
fi

if ! git rev-parse --git-dir >/dev/null 2>&1; then
  echo "Esto no es un clon de git: la estrategia de referencia está en la etiqueta $TAG. Clona el repositorio con git clone." >&2
  exit 1
fi
if ! git rev-parse -q --verify "refs/tags/$TAG" >/dev/null; then
  echo ">>> Descargando las etiquetas ($TAG) del repositorio"
  git fetch --tags origin || { echo "No pude descargar las etiquetas: ¿hay internet?" >&2; exit 1; }
fi
if ! git rev-parse -q --verify "refs/tags/$TAG" >/dev/null; then
  echo "La etiqueta $TAG no existe en el repositorio." >&2
  exit 1
fi

echo ">>> Extrayendo src/my_open de la etiqueta $TAG"
git archive "$TAG" src/my_open | tar -x
echo ">>> Listo: src/my_open ($(git rev-parse --short "$TAG^{commit}"))"

if [ "$BUILD" = 0 ]; then
  exit 0
fi
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "!!! ROS 2 Jazzy no está instalado: compila después con: colcon build --symlink-install" >&2
  exit 0
fi
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
if [ -d install/robotek_course ]; then
  echo ">>> Compilando solo my_open"
  colcon build --symlink-install --packages-select my_open
else
  echo ">>> El workspace aún no está compilado: compilo my_open y lo que necesita"
  colcon build --symlink-install --packages-up-to my_open
fi
echo ">>> Ahora: source install/setup.bash && ros2 launch robotek_course open_challenge.launch.py"
