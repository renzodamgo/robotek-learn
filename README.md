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
| `src/robotek_open` | El Open v5, la estrategia del equipo (copia de WRO-Strategies con los valores del coche) |
| `src/ldlidar_stl_ros2` | Driver del LiDAR LD19 (copia del de LDROBOT, MIT) |
| `src/rrc_lite_driver`, `src/rrc_lite_msgs` | Driver de la placa RRC Lite (basado en Hiwonder MentorPi, con permiso) |
| `tools/setup-pi.sh` | Deja un Raspberry Pi listo para el coche real (un solo comando) |
| `tools/pi-monitor` | App de la barra de menús del Mac: batería, temperatura, CPU, Wi-Fi y ROS del Pi del coche, con historial de caídas |
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

## Quiero que el coche haga la ronda ya

En la rama `main` está el andamiaje (simulador, nodos del coche, drivers); tu estrategia, el paquete `src/my_open`, la
escribes tú en la guía (git la ignora). La estrategia **terminada** de los pasos 2 a 9 se instala con
`tools/get-reference.sh` (la extrae de la etiqueta `step-9`). Con un Raspberry Pi (Ubuntu 24.04) y el coche **sobre soportes**:

```bash
git clone https://github.com/renzodamgo/robotek-learn.git && cd robotek-learn
bash tools/setup-pi.sh --with-reference        # ROS, dependencias, udev, grupos, estrategia del paso 9 y colcon build
source install/setup.bash && ros2 launch robotek_course open_challenge.launch.py
```

Si el script te añadió a grupos nuevos (`dialout`, `video`, `gpio`), cierra la sesión y vuelve a entrar antes del último comando.
Espera a que los LED de la placa se pongan **verdes** y pulsa el **botón de la placa una vez**: es el Start (`/start`) y el
coche hace las tres vueltas de la ronda Open; una segunda pulsación lo detiene (`/robotek/stop`). Ver los límites de
velocidad en `src/robotek_hardware/README.md` (con el motor 310 y los engranajes 54:30, `motor_max_rps:=6.7`). Para tu propia estrategia: crea tu paquete como en
el paso 0 de la guía y lanza `car.launch.py strategy:=<tu paquete>`. Sin `src/my_open`, el launch lo dice y sale con
código 1; `bash tools/get-reference.sh --force` reemplaza (con copia de seguridad `src/my_open.bak-FECHA`) tu paquete por el de referencia.

## Coche real (Raspberry Pi)

En el Raspberry Pi del coche (Ubuntu 24.04, arm64, con internet) basta clonar el repositorio y ejecutar un script:

```bash
git clone https://github.com/renzodamgo/robotek-learn.git
cd robotek-learn
bash tools/setup-pi.sh            # ROS 2 Jazzy base, dependencias, reglas de udev, grupos y colcon build
```

Crea tu paquete `my_open` en `src` (el comando `ros2 pkg create` de arriba) o instala el de referencia con
`bash tools/get-reference.sh` (o `setup-pi.sh --with-reference`), y vuelve a ejecutar el script (salta lo que ya está hecho). Después de cerrar la sesión y volver a entrar (por los grupos nuevos):

```bash
bash tools/setup-pi.sh --check    # placa, LiDAR, cámara, grupos: solo comprueba, no cambia nada
source install/setup.bash
ros2 launch robotek_course open_challenge.launch.py     # = car.launch.py strategy:=my_open
```

### Probar más rápido

`ros2 launch robotek_course open_challenge.launch.py button:=false` arranca solo 12 s después de lanzar
(`start_delay:=...`; con el Obstacle v3, 20 s por defecto, porque tarda 13–17 s en estar listo), sin pulsar el botón; la primera pulsación del botón ya es la parada. Ctrl+C también para el motor.

### Correr el Open v5 (la estrategia del equipo)

El Open v5 está en `src/robotek_open` y es el que corre el coche del equipo. Con el repositorio clonado y
`bash tools/setup-pi.sh` hecho, una ronda es un comando:

```bash
cd ~/robotek-learn && git pull && colcon build --symlink-install
bash tools/car-run.sh open
```

`car-run.sh` para el arranque automático y los nodos que hayan quedado (`car-clean.sh`), pone la red que ve el
ordenador (Cyclone DDS, dominio 1) y lanza `car.launch.py` con el Open v5 y los valores del coche
(`src/robotek_open/config/car_picam.yaml`). Cuando escribe `READY`, los LED están en verde: una pulsación del botón
de la placa es el Start. Imprime cada cambio de estado con su tiempo, las vueltas y el tiempo final. La telemetría
queda en `~/runs/<fecha>-<hora>-open-v5/`.

* **Engranajes.** El coche lleva engranajes 1:1 y `robot.yaml` trae `motor_rps_per_mps: 7.0` (con 8,0 el coche va un
  poco más rápido: `motor_rps_per_mps:=8.0` al lanzar). El tope del Open es `motor_max_rps` 6,7.
* **Valores del coche** (`car_picam.yaml`): `cruise_speed` 0,85, `corner_speed` 0,64, `inner_gap` 0,50,
  `turn_tolerance` 0,30, `lateral_kp` 1,0, `brake_margin` 0,35, `wall_heading_gain` 0,3, `finish_speed` 0,32,
  `finish_lead` 0,14, `direction_fallback: ccw`, y los colores de la cámara IMX708. Cualquiera se cambia al lanzar:
  `bash tools/car-run.sh open cruise_speed:=0.8`.
* **Sentido:** lo decide la cámara (la línea, azul o naranja, más larga). Si en 1 s no decide, sale en antihorario
  (`direction_fallback`), y el LiDAR lo corrige antes de la primera esquina. Sin cámara corre igual.
* **Rumbo:** el giroscopio, corregido en cada recta con la pared de la isla (`wall_heading_gain`): el giroscopio del
  coche lee un 6 % de menos.
* **Distancia:** no hay encoder de ruedas ni odometría: lo que avanza el coche en una recta es lo que se acerca la
  pared de enfrente.
* **Botón:** las pulsaciones de los primeros 5 s después del Start se ignoran; después, otra pulsación para el coche.
* **Luces:** verde = listo; azul = pulsado; rojo parpadeando = batería baja.
* `camera_always:=true` deja la cámara encendida toda la ronda (para verla o grabarla en RViz); `VERBOSE=1` delante
  muestra todo el registro.

#### Todos los parámetros del Open v5

**Cómo se cambian.** Los ocho marcados con ✓ se cambian al lanzar, sin editar nada:
`bash tools/car-run.sh open cruise_speed:=0.8 inner_gap:=0.45`. Para los demás, copia
`src/robotek_open/config/car_picam.yaml` entero, cambia lo que quieras y lánzalo con
`strategy_params:=<tu archivo>`: ese archivo **sustituye** a `car_picam.yaml`, no se suma. La columna «Código» es el
valor por defecto de `robotek_open/v5/mission.py` y `v5/settings.py`; «Coche» es el de `car_picam.yaml`, que
`car.launch` carga solo con la cámara IMX708 (vacío = el del código).

**La estrategia (la misión)**

| Parámetro | Código | Coche | Al lanzar | Qué hace |
|---|---:|---:|:-:|---|
| `cruise_speed` | 1,28 | 0,85 m/s | ✓ | velocidad en las rectas |
| `corner_speed` | 1,0 | 0,64 m/s | ✓ | velocidad en las 12 esquinas; frena hasta ella al acercarse |
| `brake_margin` | 0,15 | 0,35 m | ✓ | cuánto antes de `turn_front_distance` ya va a la velocidad de esquina (más = frena antes) |
| `inner_gap` | 0,25 | 0,50 m | ✓ | distancia a la isla en las rectas (en un pasillo estrecho guarda 0,25 m a la pared exterior) |
| `lateral_kp` | 0,3 | 1,0 | ✓ | cuánto corrige la distancia a la isla (rad por m de error, como mucho 0,4 rad) |
| `turn_tolerance` | 0,30 | 0,30 rad | ✓ | el arco termina a esta distancia de los 90°; más = esquina más corta |
| `corner_lookahead` | 0,35 | 0,35 m | ✓ | el final de la isla así de cerca (o un lado abierto) empieza la esquina |
| `turn_front_distance` | 1,19 | | | pared de enfrente (a lo largo de la pista) desde la que puede empezar una esquina |
| `wall_heading_gain` | 0,0 | 0,3 | | parte de la diferencia entre el giroscopio y la pared de la isla que corrige en cada escaneo (el giroscopio del coche lee un 6 % de menos) |
| `finish_speed` | 0,15 | 0,32 m/s | | velocidad del último tramo |
| `finish_lead` | 0,08 | 0,14 m | | frena cuando la pared de enfrente está así de cerca de la que había en la salida |
| `direction` | `auto` | | | `ccw`, `cw` o `auto` (lo decide la cámara) |
| `direction_fallback` | (vacío) | `ccw` | | sentido si la cámara no decide al segundo de calibrar; el LiDAR lo corrige antes de la primera esquina. Vacío = espera y se para |
| `laps` | 3 | | | vueltas (menos solo para pruebas en el coche) |

**El nodo**

| Parámetro | Código | Coche | Al lanzar | Qué hace |
|---|---:|---:|:-:|---|
| `scan_max_age` | 0,35 | 1,0 s | | un LiDAR más viejo que esto frena el coche hasta que vuelva |
| `imu_max_age` | 0,2 s | | | lo mismo con el giroscopio |
| `detect_crash` | `false` | `false` | ✓ | detector de choques: para el coche, reinicia la placa y guarda los últimos 5 s en `crash-<hora>.json` |
| `crash_window_s` | 5,0 s | | | segundos que guarda el detector |
| `crash_min_rays` / `crash_clear_rays` | 120 / 180 | | | un choque = escaneos con menos de 120 rayos válidos durante `crash_blind_s` (hasta uno con 180) |
| `crash_blind_s` | 0,4 s | | | |
| `crash_gyro_jump` | 3,0 rad/s | | | o un salto del giroscopio así en 0,1 s (0 = no) |
| `telemetry_path` | `auto` | | | `auto` = `~/runs/<fecha>-<hora>-open-v5/telemetry.csv`; vacío = sin telemetría |
| `telemetry_sync_seconds` | 0,5 s | | | cada cuánto se escribe al disco (por si se corta la batería) |
| `auto_start` | `false` | | | `true` = sale solo, sin el botón |
| `shutdown_on_done` / `done_hold` | `false` / 35 s | | | apagar el nodo al terminar, y cuánto espera antes |

**La cámara** (colores en HSV de OpenCV; si `h_min` es mayor que `h_max`, el rango pasa por el rojo)

| Parámetro | Código | Coche | Qué hace |
|---|---:|---:|---|
| `vision_blue_h_min` / `h_max` / `s_min` / `v_min` | 100 / 135 / 80 / 25 | 95 / 135 / 60 / 60 | la línea azul |
| `vision_orange_h_min` / `h_max` / `s_min` / `v_min` | 158 / 4 / 60 / 40 | 2 / 25 / 60 / 60 | la línea naranja |
| `vision_wall_v_max` | 40 | 70 | más oscuro que esto es pared negra |
| `vision_wall_min_rows` | 8 | | filas de pared que hacen falta para recortar por ella |
| `vision_floor_mask` | `true` | | mira solo el suelo, dentro de las paredes |
| `vision_line_contrast` / `vision_line_tint` | 0 / 8 | 30 / 8 | cuánto tiene que destacar la línea del tapete |
| `direction_confirmations` | 3 | | imágenes seguidas de acuerdo para elegir el sentido |

**El launch del coche** (`car.launch.py`; todos se pasan igual: `nombre:=valor`)

| Argumento | Para el Open | Qué hace |
|---|---|---|
| `motor_rps_per_mps` | 7,0 (`robot.yaml`) | vueltas del motor por m/s; con los engranajes 1:1, 8,0 va un poco más rápido |
| `motor_max_rps` | 6,7 (`auto`) | tope del motor |
| `motor_min_rps` | 0 | mínimo mientras el coche se mueve (0 = sin mínimo) |
| `button` / `start_delay` | `true` / 12 s | `button:=false` arranca solo a los `start_delay` s; entonces la primera pulsación es el Stop |
| `board_button` | `false` | también el botón de la RRC Lite (el del equipo es el GPIO 16) |
| `camera_always` | `false` | `true` deja la cámara encendida toda la ronda (para verla en RViz); si no, se apaga al decidir el sentido |
| `picam_jpeg_quality` | 30 (`auto`) | calidad del JPEG que ve RViz por Wi-Fi |
| `done_reset` | `DONE,CRASH` (`auto`) | estados en que se reinicia la placa para que el motor pare del todo |
| `battery_guard` / `battery_min_v` | `true` / 6,8 V | aviso de batería baja (luz roja) |
| `strategy_params` | `car_picam.yaml` (`auto`) | el archivo de valores; otro lo sustituye |
| `debug` | `false` | `true` añade el `round_reporter` (ve líneas, giros y tiempos) |

La guía, paso a paso: [Correr el Open en el Pi desde cero](https://renzodamian.com/learn/open-strategy/correr-en-el-pi/).

### Que arranque solo al encender el Pi

```bash
bash tools/car-autostart.sh on                    # el Open v5 con los valores del coche
bash tools/car-autostart.sh status                # ¿está corriendo?   ... log: su salida
bash tools/car-autostart.sh stop                  # pararlo ahora (p. ej. para lanzar otra cosa a mano)
bash tools/car-autostart.sh start                 # otra ronda: el Open termina en DONE y hay que relanzarlo
bash tools/car-autostart.sh off                   # que no arranque más al encender
```

Es un servicio de usuario de systemd (`robotek-car.service`, sin sudo). Al encender el Pi espera el Wi-Fi (hasta 60 s)
y lanza `car.launch.py` con esos argumentos; la estrategia **espera el botón de la placa**. `button:=false` no se acepta
aquí: el coche saldría solo a los 12 s de cada arranque. `on` otra vez cambia los argumentos
(`~/.config/robotek/autostart-args`). Cada arranque y cada `stop` pasan antes por `tools/car-clean.sh`, que para todos
los nodos del coche que hayan quedado de un launch anterior y deja los motores a 0 (el 4 nov 2026, nodos sobrantes
pusieron dos estrategias a la vez y frenaron el Open hasta chocar): úsalo también a mano antes de cualquier prueba.
Cada ronda queda en `~/runs/<fecha>-<hora>-autostart/`: `launch.log`, `kernel.log` (cortes del USB de la placa) y un
rosbag ligero que empieza al pulsar Start. Tras un `git pull` y
`colcon build`, el código nuevo entra en el siguiente arranque (o con `stop` y `start`).

### Conducir el coche con el teclado (para colocarlo en la salida)

En el coche, todo menos la estrategia:

```bash
ros2 launch robotek_course car.launch.py teleop:=true                  # hasta ~0,2 m/s
ros2 launch robotek_course car.launch.py teleop:=true motor_max_rps:=3   # otro tope para conducir (el de robot.yaml es 3,4 rps)
```

En tu ordenador, en la misma Wi-Fi que el coche, una ventana como un juego de carreras: mantén y combina
**↑ ↓ ← →** (o **W A S D**); al soltar todo se para y centra la dirección; si la ventana pierde el foco o se cierra,
el coche se para. Los números son la velocidad (m/s) y los grados de giro.

* **Mac** (RoboStack):

  ```bash
  ROBOSTACK_ENV=~/robotek-ros/jazzy bash tools/keyboard_racing.sh 0.4 15
  ```

* **Windows**: desde una terminal con ROS 2 Jazzy cargado (RoboStack con `pixi shell`, o los binarios oficiales con
  `call C:\dev\ros2_jazzy\local_setup.bat`):

  ```powershell
  powershell -ExecutionPolicy Bypass -File tools\keyboard_racing.ps1 -Speed 0.4 -Steer 15
  ```

  La primera vez, permite a Python en el cortafuegos de Windows (redes privadas). Si el coche tiene otra IP:
  `-PiIp 192.168.8.xxx`.

Las dos usan la red del coche (Cyclone DDS, `ROS_DOMAIN_ID` 1, descubrimiento en la subred, el Pi como par fijo). La
velocidad real la limita `motor_max_rps` del launch, no la ventana. Desde una terminal del Pi, sin ventana:
`ros2 run robotek_course keyboard_drive --ros-args -p hold:=true` (una tecla por movimiento: `u i o / j k l / m , .`).

### Ver qué hace

* `debug:=true` en el launch: la misma terminal informa de los estados, el sentido, cada esquina y vuelta y las líneas
  cruzadas.
* `python3 tools/open_monitor.py` en otra terminal del Pi: dos veces por segundo el estado, lo que ve la cámara (y las
  líneas que cruza), el LiDAR, el giroscopio y la dirección y velocidad que manda la estrategia; guarda imágenes de la
  decisión del sentido.
* `get-reference.sh` aplica los arreglos del coche de `tools/patches/` (con `--no-patch`, la referencia exacta del
  curso).

**Después de compilar en el Pi, `sync` (o `sudo poweroff`) antes de cortar la corriente**: un corte en mitad de la
escritura deja archivos vacíos (le pasó al ejecutable de la estrategia).

**La primera vez, con el coche sobre soportes (ruedas en el aire).** Ni el script ni el launch mueven el motor por su
cuenta: tu estrategia espera el Start (el botón de la placa) y el motor tiene límites de velocidad, aceleración y un
watchdog. El detalle (nodos, calibración, nombres de los dispositivos) está en
[`src/robotek_hardware/README.md`](src/robotek_hardware/README.md). El driver de la placa RRC Lite (basado en código de
Hiwonder, incluido con su permiso) está en `src/rrc_lite_*`; ver [CREDITS.md](CREDITS.md). Páginas de la guía:

* [Paso 10: el coche real](https://renzodamian.com/learn/open-strategy/paso-10-el-coche-real/)
* [Paso 11: solución de problemas](https://renzodamian.com/learn/open-strategy/paso-11-solucion-de-problemas/)
* [Paso 12: afinar](https://renzodamian.com/learn/open-strategy/paso-12-afinar/)

## El reto Obstacle

La segunda guía, **https://renzodamian.com/learn/obstacle-strategy**, construye la estrategia del reto Obstacle: tres
vueltas pasando las señales rojas por la derecha y las verdes por la izquierda, y al final aparcar en paralelo. Tu
paquete es `src/my_obstacle` (git lo ignora en `main`); la referencia está en la rama **`solutions-obstacle`**, con
una etiqueta `obstacle-step-N` por paso.

```bash
cd src
ros2 pkg create --build-type ament_python my_obstacle --node-name drive \
  --license MIT --maintainer-name "Tu Nombre" --maintainer-email tu@correo.com \
  --description "Mi estrategia del reto Obstacle" \
  --dependencies rclpy geometry_msgs sensor_msgs std_msgs cv_bridge robotek_common robotek_course
```

Una ronda en Gazebo: `seed` elige la disposición de las señales con el procedimiento de las reglas (la misma semilla
es el mismo campo) y `parking` pone el aparcamiento en la sección de salida.

```bash
ros2 launch robotek_course obstacle_round.launch.py strategy:=my_obstacle seed:=4 direction:=ccw parking:=true
ros2 run robotek_course score_obstacle                       # puntúa de nuevo la última ronda
python3 course/obstacle/verify_step.py 7                     # el paso 7 en Gazebo, sin ventana: PASS o FAIL
```

El laboratorio da la misma ronda en segundos, sin Gazebo (el campo, el coche real y el juez del Obstacle):

```bash
python3 -m robotek_evaluation.obstacle_lab --strategy my_obstacle.strategy:Strategy --seed 2 --direction cw
python3 -m robotek_evaluation.obstacle_lab --strategy my_obstacle.strategy:Strategy --layouts 0-4 --seeds 0-4 --early
python3 -m robotek_evaluation.obstacle_lab --strategy my_obstacle.strategy:Strategy --parking --layouts 0-9
```

| Pieza | Qué es |
|---|---|
| `robotek_course/obstacle_base.py` | La clase base de tu estrategia: escaneos, giroscopio, señales vistas (`on_signs`), `update` |
| `robotek_course/obstacle_vision.py` | Ver las señales: grupos del LiDAR, manchas rojas y verdes de la cámara, y su asociación |
| `robotek_evaluation/obstacle_field.py` | El campo por las reglas: las 36 cartas, el aparcamiento, el mundo de Gazebo |
| `robotek_evaluation/obstacle_judge.py` | El juez: vueltas, lados, señales movidas, parada y aparcamiento (40, o 55 con el aparcamiento) |
| `robotek_evaluation/obstacle_lab.py` | El laboratorio rápido, una ronda o lotes en paralelo |
| `robotek_course/launch/obstacle_round.launch.py` | Una ronda: campo, simulador, tu nodo y el árbitro (`obstacle_round_recorder`) |

Los puntos son estimaciones de simulación, no puntuación oficial; los +7 por arrancar dentro del aparcamiento no se
evalúan.

### El Obstacle en el coche real

La estrategia terminada del equipo (`src/robotek_obstacle_v3`) corre en el coche:

```bash
bash tools/car-autostart.sh stop && bash tools/car-clean.sh
ros2 launch robotek_course car.launch.py strategy:=robotek_obstacle_v3 camera_driver:=picam
```

Sin más argumentos, `car.launch.py` usa los valores de la mejor ronda: 4 rps, la cámara IMX708 y 17,5° de giro
(`src/robotek_obstacle_v3/config/car_picam.yaml`), y 20 s con `button:=false`. El 8 nov 2026 hizo con ellos **DONE a los
85,0 s** del Start, sin aparcamiento y sin juez; antes, 118,7 s y ~138 s. Con el
aparcamiento activo da las tres vueltas (90–115 s), pero el aparcamiento planificado no sale: al motor le falta fuerza a
0,06 m/s con todo el giro. Está a prueba el aparcamiento fijo (`park_mode: 1`). Fue con el motor en M2 sin
tipo de motor, los engranajes 54:30 y el freno de pared apagado (`wall_stop_m: 0`). La cámara IMX708, cómo grabar la
ronda, RViz, el aparcamiento y lo que falta (la salida con un pilar en la recta, la fuerza del motor a poca velocidad)
están en `src/robotek_obstacle_v3/README.md`, sección «En el coche».

## Licencia

MIT, ver [LICENSE](LICENSE). Créditos y procedencia de los archivos en [CREDITS.md](CREDITS.md).

## Quick start: sala de preguntas en vivo (`/reto`)

Un cuestionario tipo Kahoot para clase: 5 preguntas por paso, 30 segundos cada una, puntos por acierto y rapidez, y
ranking en vivo. El servidor corre en tu computadora; los alumnos solo necesitan un navegador.

### Tú (profesor): arrancar el servidor

1. Instala [Bun](https://bun.sh) (una sola vez): `curl -fsSL https://bun.sh/install | bash`
2. Desde la carpeta del repositorio:

   ```bash
   bun reto/server.ts
   ```

3. En la terminal aparecen dos direcciones. Copia las dos:
   * **La tuya (anfitrión):** `http://localhost:3000/host?key=XXXXXX`. La clave `XXXXXX` cambia cada vez que arrancas
     el servidor y solo tú la ves. No se la des a los alumnos.
   * **La de los alumnos:** `http://192.168.x.x:3000/` (el número de tu computadora en la red).
4. Abre la dirección de anfitrión en tu navegador, marca los pasos que quieres jugar (del 1 al 9) y pulsa **Crear
   sala**. Verás un **código de 4 dígitos** muy grande. Proyéctalo o díctalo.
5. Cuando todos estén en la lista, pulsa **Empezar**. Con **Siguiente** avanzas de pregunta y de paso.

### Alumnos: cómo entrar

1. Conéctate al **mismo Wi-Fi** que el profesor.
2. Abre en el navegador del celular o la laptop la **dirección de alumnos** que te dé el profesor
   (por ejemplo `http://192.168.1.25:3000/`).
3. Escribe el **código de 4 dígitos** y un **apodo** (hasta 16 letras, sin tu nombre completo) y pulsa **Entrar**.
4. Espera a que el profesor empiece. En cada pregunta tienes 30 segundos: toca la respuesta (A, B, C o D). Cuanto más
   rápido aciertes, más puntos. Arriba siempre ves tu total y tu puesto.
5. Si se te cierra la página, vuelve a entrar con el **mismo código y apodo** y recuperas tus puntos.

### Si algo no funciona

* **Los alumnos no abren la dirección:** muchos Wi-Fi de colegio aíslan los dispositivos. Activa el punto de acceso
  (hotspot) de tu celular, conecta ahí tu computadora y los alumnos, y usa la dirección nueva que imprime el servidor.
* **Alumnos a distancia:** usa un túnel, por ejemplo `cloudflared tunnel --url http://localhost:3000`, y comparte el
  enlace `https://...trycloudflare.com` que aparece. No hace falta cuenta.
* **Reinicias el servidor:** la sala se pierde (todo está en memoria). Crea una nueva.
* Detalles, formato de las preguntas y privacidad: [`reto/README.md`](reto/README.md).
