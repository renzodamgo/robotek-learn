# Copia incluida de ldlidar_stl_ros2

Esta carpeta es una copia del driver del LiDAR LDROBOT LD19 (y LD06 / STL27L), incluida aquí para que
`git clone` + `colcon build` baste en el Raspberry Pi.

* Origen: https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2, commit `bf668a89baf722a787dadc442860dcbf33a82f5a`
  (8 may 2023, "Update README.md").
* Licencia: MIT, Copyright (c) 2020 SHENZHEN LDROBOT CO., LTD. El archivo `LICENSE` y los avisos de copyright de cada
  archivo están sin tocar.
* Cambios respecto al original: solo los dos arreglos de compilación para Ubuntu 24.04 / ROS 2 Jazzy de
  `../robotek_hardware/patches/ldlidar_stl_ros2-build.patch` ya aplicados (`find_package(Threads)` y
  `Threads::Threads` en `CMakeLists.txt`; `#include <pthread.h>` en `ldlidar_driver/src/logger/log_module.cpp`).
  Y un tercer cambio: en sistemas que no son Linux (macOS, solo simulación) el `CMakeLists.txt` no compila nada,
  para que `colcon build` no falle allí. No se quitó ni añadió nada más.
* No se copió el directorio `.git` del original.
