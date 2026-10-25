# robotek-learn

Un coche, un campo y una guía: aprende a programar la estrategia del **reto Open de la WRO 2026 (Future Engineers)**
con ROS 2 Jazzy y Gazebo, escribiendo tú el código paso a paso. Lo que escribes es un nodo de ROS que lee el LiDAR, el
giroscopio y la cámara de un coche simulado con las medidas y los retrasos del coche real, da tres vueltas al campo y
se detiene en la sección de salida. Al final, el mismo nodo corre en el coche de verdad (Raspberry Pi).

La guía completa, con gráficos interactivos y un simulador en el navegador, está en
**https://renzodamian.com/learn/open-strategy**. Este repositorio es el andamiaje.

## Empezar

```bash
git clone https://github.com/renzodamgo/robotek-learn.git
cd robotek-learn
bash tools/install-ubuntu.sh        # Ubuntu 24.04 o WSL2; en macOS ver la guía (RoboStack)
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robotek_course sim.launch.py          # el coche en Gazebo
```

Después crea tu paquete y sigue los pasos de la guía:

```bash
cd src
ros2 pkg create --build-type ament_python my_open --node-name drive \
  --license MIT --maintainer-name "Tu Nombre" --maintainer-email tu@correo.com \
  --description "Mi estrategia del reto Open" \
  --dependencies rclpy geometry_msgs sensor_msgs std_msgs visualization_msgs cv_bridge robotek_common robotek_course
```

## Qué hay aquí

| Carpeta | Para qué |
|---|---|
| `src/robotek_description` | El coche (URDF y mallas) |
| `src/robotek_gazebo` | El campo, el tapete y el puente con ROS 2 |
| `src/robotek_common` | Medidas del coche y odometría con LiDAR |
| `src/robotek_evaluation` | Solo simulación: juez de reglas, modelo del servo y del motor, contactos, Open lab |
| `src/robotek_course` | Lanzadores del simulador y de una ronda, el árbitro, `score`, la clase base de la estrategia |
| `src/robotek_hardware` | Los nodos del coche real (Raspberry Pi) |
| `course/tests` | Pruebas de cada paso (`pytest`, sin ROS ni Gazebo) |
| `course/verify_step.py` | Comprobación de cada paso en Gazebo |

Tu código vive en `src/my_open` (git lo ignora en `main`). La rama **`solutions`** tiene un commit y una etiqueta
`step-N` por paso para que compares:

```bash
bash tools/compare-step.sh 2 3     # qué cambia la referencia entre dos pasos
bash tools/compare-step.sh 3        # tu árbol contra la referencia del paso 3
```

## Una ronda

```bash
ros2 launch robotek_course round.launch.py strategy:=my_open gui:=true direction:=ccw
ros2 run robotek_course score                # puntúa de nuevo la última ronda
```

Las rondas se guardan en `~/robotek_runs/`. Los puntos son una **estimación de simulación** con las reglas
publicadas, no una puntuación oficial.

## Licencia

MIT, ver [LICENSE](LICENSE). Créditos y procedencia de los archivos en [CREDITS.md](CREDITS.md).
