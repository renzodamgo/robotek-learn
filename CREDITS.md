# Créditos

- **Coche y código de simulación**: Renzo Damian y el equipo Robotek (WRO 2026, Future Engineers).
- **Mallas del coche** (`src/robotek_description/meshes/*.stl`): diseño del equipo Robotek, publicadas con su permiso.
- **Textura del tapete** (`src/robotek_gazebo/models/robotek_game_mat/materials/textures/mat.png`): dibujo propio,
  generado con `tools/make_mat_texture.py`. No contiene arte ni logotipos de la WRO. Las medidas del campo siguen
  las reglas públicas del reto.
- **Herramientas**: [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/), [Gazebo Harmonic](https://gazebosim.org/),
  [ros_gz](https://github.com/gazebosim/ros_gz), [RoboStack](https://robostack.github.io/) para macOS.
- **Driver del LiDAR LD19** (`src/ldlidar_stl_ros2/`): copia del
  [ldlidar_stl_ros2 de LDROBOT](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2) (commit `bf668a8`), licencia MIT,
  Copyright (c) 2020 SHENZHEN LDROBOT CO., LTD. Su `LICENSE` y los avisos de cada archivo están intactos; solo se
  aplicaron los dos arreglos de compilación de `src/robotek_hardware/patches/ldlidar_stl_ros2-build.patch`
  (ver `src/ldlidar_stl_ros2/NOTICE-ROBOTEK.md`).
- **Driver de la placa RRC Lite** (`src/rrc_lite_driver/`, `src/rrc_lite_msgs/`): basado en
  [Hiwonder MentorPi](https://github.com/Hiwonder/MentorPi) (commit `fb6d9969`). Incluido con el permiso de Hiwonder,
  confirmado por el autor del repositorio. El código original no declara licencia; los cambios del equipo Robotek
  están bajo MIT. Ver el `NOTICE.md` de cada carpeta.
- **Cámara del coche real**: el nodo `camera_publisher` de `src/robotek_hardware` es del equipo (MIT).

WRO® es una marca de World Robot Olympiad Association. Este proyecto no está afiliado a la WRO.
