# robotek_gazebo

El campo del reto Open de la WRO 2026 para **Gazebo Harmonic** (gz-sim 8) y el puente con ROS 2.
El coche está en `robotek_description`.

```
robotek_gazebo/
├── worlds/        robotek_open_field.sdf (el campo por defecto) y siete variantes de ancho de pasillo
│                  (square100, square180, rect100x180, rect180x100, shift140, east140x100, ...)
├── models/robotek_game_mat/   el tapete; su textura se dibuja con tools/make_mat_texture.py
├── config/        robotek_bridge.yaml: los temas que cruzan entre Gazebo y ROS 2
└── launch/        robotek_sim.launch.py: Gazebo + puente + coche
```

Los campos llevan un sensor de contacto (`wall_contact`) en los muros: el juez lo usa para saber si el coche los tocó.

Normalmente no lo lanzas directamente: usa `ros2 launch robotek_course sim.launch.py` (el simulador) o
`ros2 launch robotek_course round.launch.py` (una ronda con puntuación).
