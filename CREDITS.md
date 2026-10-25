# Créditos

- **Coche y código de simulación**: Renzo Damian y el equipo Robotek (WRO 2026, Future Engineers).
- **Mallas del coche** (`src/robotek_description/meshes/*.stl`): diseño del equipo Robotek, publicadas con su permiso.
- **Textura del tapete** (`src/robotek_gazebo/models/robotek_game_mat/materials/textures/mat.png`): dibujo propio,
  generado con `tools/make_mat_texture.py`. No contiene arte ni logotipos de la WRO. Las medidas del campo siguen
  las reglas públicas del reto.
- **Herramientas**: [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/), [Gazebo Harmonic](https://gazebosim.org/),
  [ros_gz](https://github.com/gazebosim/ros_gz), [RoboStack](https://robostack.github.io/) para macOS.
- **Controladores del hardware real** (no incluidos): la placa de motores usa el controlador de
  [Hiwonder MentorPi](https://github.com/Hiwonder/MentorPi) y el LiDAR LD19 el de
  [LDROBOT](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2) (MIT). Ver `src/robotek_hardware/README.md`.

WRO® es una marca de World Robot Olympiad Association. Este proyecto no está afiliado a la WRO.
